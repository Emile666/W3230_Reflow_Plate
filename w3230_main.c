/*==================================================================
  File Name    : w3230_main.c
  Author       : Emile
  ------------------------------------------------------------------
  Purpose : This files contains the main() function and all the 
            hardware related functions for the STM8S105C6T6 uC.
            It is meant for the custom-build W3230 hardware.
  ------------------------------------------------------------------
  This file is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  This is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this file.  If not, see <http://www.gnu.org/licenses/>.
  ==================================================================
*/ 
#include <intrinsics.h> 
#include <stdio.h>
#include "w3230_main.h"
#include "w3230_lib.h"
#include "scheduler.h"
#include "adc.h"
#include "eep.h"
#include "i2c_bb.h"
#include "comms.h"
#include "uart.h"
#include "spi.h"
#include "max6675.h"
#include "misc.h"

// Version number for W3230 firmware
char version[] = "W3230-Reflow V0.10\n";

// Global variables
int16_t   temp_tc;           // The temperature in °C from the thermocouple
int16_t   temp_tf;           // The temperature in °F from the thermocouple
uint8_t   mpx_nr = STD_CA1;  // Used in multiplexer() function
int16_t   pwr_on_tmr = 1000; // Needed for 7-segment display test
bool      pid_sw = false;    // Switch for pid_out
int16_t   pid_fx = 0;        // Fix-value for pid_out
ma        tc_ma;             // struct for thermocouple MA filter temperature

// External variables, defined in other files
extern bool     no_tc;            // 1 = no thermocouple present
extern bool     err_id;           // 1 = Device ID bit is not 0
extern bool     run_profile;      // True = start Profile run
extern uint8_t  top_10, top_1, top_01; // values of 10s, 1s and 0.1s
extern uint8_t  bot_10, bot_1, bot_01; // values of 10s, 1s and 0.1s
extern uint8_t  sensor2_selected; // DOWN button pressed < 3 sec. shows 2nd temperature / pid_output
extern bool     menu_is_idle;     // No menus in STD active
extern bool     fahrenheit;       // false = Celsius, true = Fahrenheit
extern int16_t  setpoint;         // local copy of SP variable
extern uint8_t  ts;               // Parameter value for sample time [sec.]
extern int16_t  pid_out;          // Output from PID controller in E-1 %
extern uint32_t t2_millis;        // needed for delay_msec()
extern uint8_t  rs232_inbuf[];
extern uint8_t  std_tc;           // State for Temperature Control

/*-----------------------------------------------------------------------------
  Purpose  : This routine sets all port C and D pins connected to the 7-segment
             display to inputs with pull-up resistors disabled.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void setAsInputs(void)
{
    PC_ODR     &= ~PC_SSD;  // All outputs are 0
    PD_ODR     &= ~PD_SSD;  // All outputs are 0
    PC_DDR     &= ~PC_SSD;  // Set pins to input
    PD_DDR     &= ~PD_SSD;  // Set pins to input
    PC_CR1     &= ~PC_SSD;  // Disable pull-ups
    PD_CR1     &= ~PD_SSD;  // Disable pull-ups
} // setAsInputs()

/*-----------------------------------------------------------------------------
  Purpose  : This routine sets the following port lines connected to the 
             7-segment display to push-pull outputs:
             - port C pins denoted by maskC
             - port D pins denoted by maskD
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void setAsOutputs(uint8_t maskC, uint8_t maskD)
{
    PC_CR1     |= maskC; // Set to push-pull
    PD_CR1     |= maskD; // Set to push-pull
    PC_DDR     |= maskC; // Set pins to output
    PD_DDR     |= maskD; // Set pins to output
} // setAsOutputs()

/*-----------------------------------------------------------------------------
  Purpose  : This routine multiplexes the 6 segments of the 7-segment displays.
             It runs at 1 kHz, so full update frequency is 166 Hz.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void multiplexer(void)
{
    uint8_t bt;
    uint8_t pc_msk,pd_msk;
    
    setAsInputs();  // Disable all 7-segment LEDs and CC/CA pins and set to 0
    switch (mpx_nr)
    {
        //---------------------------------------------------------------
        // Display 1 CA (C2A1,PC4) is the 10s digit top display
        //---------------------------------------------------------------
        //  a    b    c   d    e   f   g  dp  (top_10) 
        // C4A3 C6A5 cba CXAX edc fed gfe pgf
        // PC3  PC2  PD7 PC1  PD4 PD3 PD2 PD0
        //---------------------------------------------------------------
        case STD_CA1: // From C2A1 to non-CA/CC lines: 1C,1E,1D,1G,1DP
            bt      = top_10;                // top_10 is source for display 1
            pc_msk  = C2A1 | ((bt & 0xC0) >> 4) | ((bt & 0x10) >> 3);
            pd_msk  = ((bt & 0x20) << 2) | ((bt & 0x0E) << 1) | (bt & 0x01);
            bt      = ~bt; // C2A1 is 1, so bits needs to be inverted for LED on
            // Set LEDs from C2A1 to non-CA/CC lines
            CBAb    = ((bt & 0x20) == 0x20); // set cba=c
            PD_ODR |= ((bt & 0x0E) << 1);    // set edc=e,fed=f,gfe=g
            PGFb    = ((bt & 0x01) == 0x01); // set pgf=dp
            // Set LEDs from C2A1 to other CA/CC lines
            PC_ODR |= ((bt & 0xC0) >> 4);    // set C4A3=a,C6A5=b
            CXAXb   = ((bt & 0x10) == 0x10); // set CXAX=d
            C2A1b   = 1;                     // Enable Common-Anode 1
            mpx_nr  = STD_CA3;               // next state
            break;

        //---------------------------------------------------------------
        // Display 3 CA (C4A3,PC3) is the 0.1s digit top display
        //---------------------------------------------------------------
        //  a    b    c   d    e   f   g  dp   (top_01)
        // C6A5 cba CXAX edc fed gfe pgf  -
        // PC2  PD7 PC1  PD4 PD3 PD2 PD0  -
        //---------------------------------------------------------------
        case STD_CA3: // From C4A3 to non-CA/CC lines: 3B,3D,3E,3F,3G
            bt      = top_01; // top_01 is source for display 3
            pc_msk  = C4A3 | ((bt & 0x80) >> 5) | ((bt & 0x20) >> 4);
            pd_msk  = ((bt & 0x40) << 1) | (bt & 0x1C) | ((bt & 0x02) >> 1);
            bt      = ~bt;    // CA3 is 1, so bits needs to be inverted for LED on
            // Set LEDs from C4A3 to non-CA/CC lines
            CBAb    = ((bt & 0x40) == 0x40); // set cba=b
            PD_ODR |=  (bt & 0x1C);          // set edc=d,fed=e,gfe=f
            PGFb    = ((bt & 0x02) == 0x02); // set pgf=g
            // Set LEDs from C4A3 to other CA/CC lines
            C6A5b   = ((bt & 0x80) == 0x80); // set C6A5=a
            CXAXb   = ((bt & 0x20) == 0x20); // set CXAX=c
            C4A3b   = 1;                     // Enable Common-Anode 3
            mpx_nr  = STD_CA5;               // next state
            break;
            
        //---------------------------------------------------------------
        // Display 5 CA (C6A5,PC2) is the 1s digit bottom display
        //---------------------------------------------------------------
        //  7    6    5   4    3   2   1  0
        //  a    b    c   d    e   f   g  dp (bot_1)
        // cba CXAX  edc fed gfe  pgf  -  -
        // PD7 PC1   PD4 PD3 PD2  PD0  -  -
        //---------------------------------------------------------------
        case STD_CA5: // From C6A5 to non-CA/CC lines: 5A,5C,5D,5E,5F
            bt      = bot_1; // bot_1 is source for display 5
            pc_msk  = C6A5 | ((bt & 0x40) >> 5);
            pd_msk  = (bt & 0x80) | ((bt & 0x38) >> 1) | ((bt & 0x04) >> 2);
            bt      = ~bt;   // CA5 is 1, so bits needs to be inverted for LED on
            // Set LEDs from C6A5 to non-CA/CC lines
            CBAb    = ((bt & 0x80) == 0x80); // set cba=a
            PD_ODR |= ((bt & 0x38) >> 1);    // set edc=c,fed=d,gfe=e
            PGFb    = ((bt & 0x04) == 0x04); // set pgf=f
            // Set LEDs from C4A3 to other CA/CC lines
            CXAXb   = ((bt & 0x40) == 0x40); // set CXAX=b
            C6A5b   = 1;                     // Enable Common-Anode 5
            mpx_nr  = STD_CAX;               // next state
            break;
            
        //---------------------------------------------------------------
        // The last CA line (CXAX,PC1) contains leds 3dp, 5g and 5dp.
        //---------------------------------------------------------------
        // dp3  g5 dp5 
        // edc fed gfe
        // PD4 PD3 PD2
        //---------------------------------------------------------------
        case STD_CAX:
            pc_msk = CXAX;
            pd_msk = ((top_01 & 0x01) << 4) | ((bot_1 & 0x03) << 2);
            EDCb   = ((top_01 & 0x01) != 0x01); // set edc=!3dp
            FEDb   = ((bot_1  & 0x02) != 0x02); // set fed=!5g
            GFEb   = ((bot_1  & 0x01) != 0x01); // set gfe=!5dp
            CXAXb  = 1;                         // Enable Common-Anode X
            mpx_nr = STD_CC2;                   // Next state: display 2 control
            break;

        //---------------------------------------------------------------
        // Display 2 CC (C2A1,PC4) is the 1s digit top display
        //---------------------------------------------------------------
        //  a    b    c   d    e   f   g  dp  (top_1) 
        // C4A3 C6A5 cba CXAX edc fed gfe pgf
        // PC3  PC2  PD7 PC1  PD4 PD3 PD2 PD0
        //---------------------------------------------------------------
        case STD_CC2: // From non-CA/CC lines to C2A1: 2C,2E,2F,2G,2DP
            bt      = top_1; // top_1 is source for display 2
            pc_msk  = C2A1 | ((bt & 0xC0) >> 4) | ((bt & 0x10) >> 3);
            pd_msk  = ((bt & 0x20) << 2) | ((bt & 0x0E) << 1) | (bt & 0x01); 
            // Set LEDs from C2A1 to non-CA/CC lines
            CBAb    = ((bt & 0x20) == 0x20); // set cba=c
            PD_ODR |= ((bt & 0x0E) << 1);    // set edc=e,fed=f,gfe=g
            PGFb    = ((bt & 0x01) == 0x01); // set pgf=dp
            // Set LEDs from C2A1 to other CA/CC lines
            PC_ODR |= ((bt & 0xC0) >> 4);    // set C4A3=a,C6A5=b
            CXAXb   = ((bt & 0x10) == 0x10); // set CXAX=d
            C2A1b   = 0;                     // Enable Common-Cathode 1
            mpx_nr  = STD_CC4;               // next state
            break;
            
        //---------------------------------------------------------------
        // Display 4 CC (C4A3,PC3) is the 10s digit bottom display
        //---------------------------------------------------------------
        //  a    b    c   d    e   f   g  dp   (bot_10)
        // C6A5 cba CXAX edc fed gfe pgf  -
        // PC2  PD7 PC1  PD4 PD3 PD2 PD0  -
        //---------------------------------------------------------------
        case STD_CC4: // From non-CA/CC lines to C4A3: 4B,4D,4E,4F,4G
            bt      = bot_10; // bot_10 is source for display 4
            pc_msk  = C4A3 | ((bt & 0x80) >> 5) | ((bt & 0x20) >> 4);
            pd_msk  = ((bt & 0x40) << 1) | (bt & 0x1C) | ((bt & 0x02) >> 1);
            // Set LEDs from C4A3 to non-CA/CC lines
            CBAb    = ((bt & 0x40) == 0x40); // set cba=b
            PD_ODR |=  (bt & 0x1C);          // set edc=d,fed=e,gfe=f
            PGFb    = ((bt & 0x02) == 0x02); // set pgf=g
            // Set LEDs from C4A3 to other CA/CC lines
            C6A5b   = ((bt & 0x80) == 0x80); // set C6A5=a
            CXAXb   = ((bt & 0x20) == 0x20); // set CXAX=c
            C4A3b   = 0;                     // Enable Common-Cathode 3
            mpx_nr  = STD_CC6;               // next state
            break;
            
        //---------------------------------------------------------------
        // Display 6 CC (C6A5,PC2) is the 0.1s digit bottom display
        //---------------------------------------------------------------
        //  7    6    5   4    3   2   1  0
        //  a    b    c   d    e   f   g  dp (bot_01)
        // cba CXAX  edc fed gfe  pgf  -  -
        // PD7 PC1   PD4 PD3 PD2  PD0  -  -
        //---------------------------------------------------------------
        case STD_CC6: // From non-CA/CC lines to C6A5: 6A,6C,6D,6E,6F
            bt      = bot_01; // bot_01 is source for display 6
            pc_msk  = C6A5 | ((bt & 0x40) >> 5);
            pd_msk  = (bt & 0x80) | ((bt & 0x38) >> 1) | ((bt & 0x04) >> 2);
            // Set LEDs from C6A5 to non-CA/CC lines
            CBAb    = ((bt & 0x80) == 0x80); // set cba=a
            PD_ODR |= ((bt & 0x38) >> 1);    // set edc=c,fed=d,gfe=e
            PGFb    = ((bt & 0x04) == 0x04); // set pgf=f
            // Set LEDs from C4A3 to other CA/CC lines
            CXAXb   = ((bt & 0x40) == 0x40); // set CXAX=b
            C6A5b   = 0;                     // Enable Common-Cathode 5
            mpx_nr  = STD_CCX;               // next state
            break;
            
        //---------------------------------------------------------------
        // The last CC line (CXAX,PC1) contains leds 4dp, 6g and 6dp.
        //---------------------------------------------------------------
        // dp4  g6 dp6 
        // edc fed gfe
        // PD4 PD3 PD2
        //---------------------------------------------------------------
        case STD_CCX:
            pc_msk = CXAX;
            pd_msk = ((bot_10 & 0x01) << 4) | ((bot_01 & 0x03) << 2);
            EDCb   = ((bot_10 & 0x01) == 0x01); // set edc=dp4
            FEDb   = ((bot_01 & 0x02) == 0x02); // set fed=g6
            GFEb   = ((bot_01 & 0x01) == 0x01); // set gfe=dp6
            CXAXb  = 0;                         // Enable Common-Cathode X
            mpx_nr = STD_CA1;                   // Back to display 1
            break;
    } // switch            
    setAsOutputs(pc_msk,pd_msk); // Enable only outputs that are active in a state
} // multiplexer()

/*-----------------------------------------------------------------------------
  Purpose  : This is the interrupt routine for the Timer 2 Overflow handler.
             It runs at 1 kHz and drives the scheduler and the multiplexer.
             Measured timing: 1.0 msec and 9 usec duration (9.1 %).
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
#pragma vector=TIM2_OVR_UIF_vector
__interrupt void TIM2_UPD_OVF_IRQHandler(void)
{
    PA_ODR |= ISR_OUT; // Time-measurement interrupt routine
    t2_millis++;       // update millisecond counter
    scheduler_isr();   // Run scheduler interrupt function
    
    if (pwr_on_tmr > 0)
    {	// 7-segment display test for 1 second
        pwr_on_tmr--;
        top_10 = top_1 = top_01 = LED_ON;
        bot_10 = bot_1 = bot_01 = LED_ON;
    } // else if
    multiplexer();        // Run multiplexer for Display
    PA_ODR   &= ~ISR_OUT; // Time-measurement interrupt routine
    TIM2_SR1_UIF = 0;     // Reset interrupt (UIF bit) so it will not fire again straight away.
} // TIM2_UPD_OVF_IRQHandler()

/*-----------------------------------------------------------------------------
  Purpose  : This routine initialises the system clock to run at 16 MHz.
             It uses the internal HSI oscillator.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void initialise_system_clock(void)
{
    CLK_ICKR       = 0;           //  Reset the Internal Clock Register.
    CLK_ICKR_HSIEN = 1;           //  Enable the HSI.
    while (CLK_ICKR_HSIRDY == 0); //  Wait for the HSI to be ready for use.
    CLK_CKDIVR     = 0;           //  Ensure the clocks are running at full speed.
 
    // The datasheet lists that the max. ADC clock is equal to 6 MHz (4 MHz when on 3.3V).
    // Because fMASTER is now at 16 MHz, we need to set the ADC-prescaler to 4.
    ADC_CR1_SPSEL = 0x02;         //  Set prescaler (SPSEL bits) to 4, fADC = 4 MHz
    CLK_SWIMCCR   = 0x00;         //  Set SWIM to run at clock / 2.
    CLK_SWR       = 0xE1;         //  Use HSI as the clock source.
    CLK_SWCR      = 0;            //  Reset the clock switch control register.
    CLK_SWCR_SWEN = 1;            //  Enable switching.
    while (CLK_SWCR_SWBSY != 0);  //  Pause while the clock switch is busy.
} // initialise_system_clock()

/*-----------------------------------------------------------------------------
  Purpose  : This routine initialises Timer 2 to generate a 1 kHz interrupt.
             16 MHz / (16 * 1000) = 1000 Hz (1000 = 0x03E8)
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void setup_timer2(void)
{
    TIM2_PSCR    = 0x04; //  Prescaler = 16
    TIM2_ARRH    = 0x03; //  High byte of 1000
    TIM2_ARRL    = 0xE8; //  Low  byte of 1000
    TIM2_IER_UIE = 1;    //  Enable the update interrupts, disable all others
    TIM2_CR1_CEN = 1;    //  Finally enable the timer
} // setup_timer2()

/*-----------------------------------------------------------------------------
  Purpose  : This routine initialises all the GPIO pins of the STM8 uC.
             See header-file for a pin-connections and functions.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void setup_gpio_ports(void)
{
    PA_DDR     |=  (SSR | BUZZER | ISR_OUT); // Set as output
    PA_CR1     |=  (SSR | BUZZER | ISR_OUT); // Set to Push-Pull
    PA_ODR     &= ~(SSR | BUZZER | ISR_OUT); // Disable PORTA outputs
    
    PB_DDR     |=  PB_LEDS; // Set blue and red LED as outputs
    PB_CR1     |=  PB_LEDS; // Set to Push-Pull
    PB_ODR     &= ~PB_LEDS; // Disable LEDs
    PB_DDR     &= ~PB_KEYS; // Set all keys as input
    PB_CR1     |=  PB_KEYS; // Enable pull-ups
    
    PC_ODR     &= ~PC_SSD; // Disable all 7-segment output lines
    PC_DDR     |=  PC_SSD; // Set 7-segment lines as output
    PC_CR1     |=  PC_SSD; // Set 7-segment/key pins to Push-Pull
    PC_ODR     &= ~PC_SSD; // Disable all 7-segment output lines
    
    PD_ODR     &= ~PD_SSD; // Disable all 7-segment output lines
    PD_DDR     |=  PD_SSD; // Set 7-segment lines as output
    PD_CR1     |=  PD_SSD; // Set 7-segment/key pins to Push-Pull
    PD_ODR     &= ~PD_SSD; // Turn off all 7-segment/key pins
    
    PE_ODR     |=  (I2C_SCL | I2C_SDA | nCS); // Must be set here, or I2C will not work
    PE_DDR     |=  (I2C_SCL | I2C_SDA | nCS); // Set as outputs
    PE_CR1     |=  nCS;                       // Set to Push-Pull
    PE_ODR     |=  nCS;                       // Disable chip-select of MAX6675
} // setup_output_ports()

/*-----------------------------------------------------------------------------
  Purpose  : This task is called every 500 msec. and processes the temperature
             from the thermocouple, connected to the MAX6675
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void adc_task(void)
{
  int16_t temp;
  
  temp   = max6675_read(); // read thermocouple temperature (Q10.2 format)
  temp  += 2;              // round up
  temp >>= 2;              // temp_tc is now in °C
  
  temp_tc = (int16_t)(moving_average(&tc_ma,(float)temp) + 0.5);  
  conv_fahrenheit();       // convert temp_tc °C to temp_tf °F
} // adc_task()

/*-----------------------------------------------------------------------------
  Purpose  : This task is called every 100 msec. and creates a slow PWM signal
             from pid_output: T = 12.5 seconds. This signal can be used to
             drive a Solid-State Relay (SSR).
  Variables: pid_out (global) is used
  Returns  : -
  ---------------------------------------------------------------------------*/
void pid_to_time(void)
{
    static uint8_t std_ptt = 1; // state [on, off]
    static uint8_t ltmr    = 0; // #times to set SSR to 0
    static uint8_t htmr    = 0; // #times to set SSR to 1
    uint16_t x;                 // temp. variable
     
    x = pid_out;
    switch (std_ptt)
    {
        case 0: // OFF
            if (ltmr == 0)
            {   // End of low-time
                htmr = (uint8_t)x; // htmr = 1.25 * pid_out
                if (htmr > 0) std_ptt = 1;
            } // if
            else ltmr--; // decrease timer
            SSRb    = 0; // SSR output = 0
            LD_REDb = 0; // red LED
            break;
        case 1: // ON
            if (htmr == 0)
            {   // End of high-time
                ltmr = (uint8_t)(100 - x); // ltmr = 100 - pid_out
                if (ltmr > 0) std_ptt = 0;
            } // if
            else htmr--; // decrease timer
            SSRb    = 1; // SSR output = 1
            LD_REDb = 1; // red LED
            break;
    } // switch
} // pid_to_time()

/*-----------------------------------------------------------------------------
  Purpose  : This task is called every 100 msec. and reads the buttons, runs
             the STD and updates the SSR slow-PWM from the PID-output.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void std_task(void)
{
    read_buttons(); // reads the buttons keys, result is stored in _buttons
    menu_fsm();     // Finite State Machine menu
    pid_to_time();  // Make Slow-PWM signal and send to SSR output-port
} // std_task()

/*-----------------------------------------------------------------------------
  Purpose  : This task converts the temperature in temp_tc, which is in °C,
             to a temperature in °F.
             Formula used: F = (9/5) * C + 32 = (18C + 320)/10
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void conv_fahrenheit(void)
{
    int16_t x = temp_tc;
    
    x <<= 4;             // x = 16 * temp_tc
    x += (temp_tc << 1); // x = 18 * temp_tc
    x += 320;            // x = 18 * temp_tc + 320
    temp_tf = divu10(x); // x = 1.8 * temp_tc + 32 = (9/5) * temp_tc + 32
} // conv_fahrenheit()

/*-----------------------------------------------------------------------------
  Purpose  : This task is called every second and contains the main control
             task for the device. It also calls temperature_control() / 
             pid_ctrl() and one_wire_task().
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void ctrl_task(void)
{
   int16_t temp;
   
    if (eeprom_read_config(EEADR_MENU_ITEM(CF))) // true = Fahrenheit
    {
      fahrenheit = true;
      temp       = temp_tf;
    } // if
    else 
    {
      fahrenheit = false;
      temp       = temp_tc;
    } // else

   if (no_tc || err_id)
   {
       //ALARM_ON;   // enable the piezo buzzer
       if (menu_is_idle)
       {  // Make it less anoying to nagivate menu during alarm
          top_10 = LED_A;
	  top_1  = LED_L;
	  top_01 = LED_A;
          bot_10 = LED_S;
          if (no_tc) { bot_1 = LED_n; bot_01 = LED_o; }
          else       { bot_1 = LED_I; bot_01 = LED_d; }
       } // if
   } else {
       //ALARM_OFF;   // reset the piezo buzzer
       ts        = eeprom_read_config(EEADR_MENU_ITEM(Ts));  // Read Ts [seconds]
       if (ts == 0) // PID Ts parameter is 0?
            pid_out = 0;          // Disable PID-output
       else pid_control(temp);    // Run PID controller
       LD_BLUEb = (run_profile == true); // Blue LED = run_profile is active
       // --------- Manual switch/fix for pid-output --------------------
       if (pid_sw) pid_out = pid_fx; // can be set via UART

       if (menu_is_idle)          // show temperature if menu is idle
       {
           {
               switch (sensor2_selected)
               {
                case 0:
                     value_to_led(temp    ,DECIMAL0,ROW_TOP); // display temperature on top row
                     value_to_led(setpoint,DECIMAL0,ROW_BOT); // display setpoint on bottom row
                     break;
                case 1:
                     value_to_led(pid_out,DECIMAL0,ROW_TOP); // display pid-output on top row
                     bot_10 = LED_P; bot_1 = LED_I; bot_01 = LED_d;
                     break;
               } // switch
           } // else
       } // if
   } // else
} // ctrl_task()

/*-----------------------------------------------------------------------------
  Purpose  : This task is called every second and updates the
             current running temperature profile.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void prfl_task(void)
{
    static uint8_t sec = 0;
    char  s2[25];
        
    update_profile(); 
    // Logging to UART every 5 seconds
    if (++sec >= 5)
    {   // every 5 seconds
        sec = 0;
        sprintf(s2,"l%d %d %d\n",std_tc,fahrenheit ? temp_tf : temp_tc,setpoint);
        xputs(s2);
    } // if
} // prfl_task();

/*-----------------------------------------------------------------------------
  Purpose  : This is the main entry-point for the entire program.
             It initialises everything, starts the scheduler and dispatches
             all tasks.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
int main(void)
{
    __disable_interrupt();
    initialise_system_clock(); // Set system-clock to 16 MHz
    setup_gpio_ports();        // Init. needed output-ports for LED and keys
    setup_timer2();            // Set Timer 2 to 1 kHz
    spi_init();                // Init. SPI bus
    uart_init();               // Init. serial communication
    
    init_moving_average(&tc_ma,5,20.0);
    // Initialise all tasks for the scheduler
    scheduler_init();                    // clear task_list struct
    add_task(adc_task ,"ADC",  0, 1000); // every second
    add_task(std_task ,"STD", 50,  100); // every 100 msec.
    add_task(ctrl_task,"CTL",200, 1000); // every second
    add_task(prfl_task,"PRF",300, 1000); // every second
    __enable_interrupt();
    xputs(version); // print version number
    
    while (1)
    {   // background-processes
        dispatch_tasks();     // Run task-scheduler()
        switch (rs232_command_handler()) // run command handler continuously
        {
            case ERR_CMD: xputs("Cmd Error\n"); break;
            case ERR_NUM: xputs("Num Error\n");  break;
            default     : break;
        } // switch
    } // while
} // main()
