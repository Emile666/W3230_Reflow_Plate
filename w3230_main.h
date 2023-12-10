/*==================================================================
  File Name    : w3230_main.h
  Author       : Emile
  ------------------------------------------------------------------
  This is the header file for w3230_main.c, which is the main-body
  W3230 temperature controller. This version is made for the STM8S105C6T6 uC.
 
  This file is part of the W3230 project.
  ------------------------------------------------------------------
  This is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  This is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this file.  If not, see <http://www.gnu.org/licenses/>.
  ------------------------------------------------------------------
  Schematic of the connections to the MCU.
 
                        STM8S105C6T6 HW version 01
      MCU pin-name            Function |    MCU pin-name        Function
   ------------------------------------|--------------------------------
   01 NRST                    !RST     | 13 VDDA
   02 PA1/OSC                 -        | 14 VSSA
   03 PA2/OSCOUT              -        | 15 PB7/AIN7            UP
   04 VSSIO_1                          | 16 PB6/AIN6            PWR
   05 VSS                              | 17 PB5/AIN5[I2C_SDA]   DOWN
   06 VCAP                             | 18 PB4/AIN4[I2C_SCL]   SET
   07 VDD                              | 19 PB3/AIN3[TIM1_ETR]  LD3_RED 
   08 VDDIO_1                          | 20 PB2/AIN2[TIM1_CH3N] LD2_BLUE
   09 PA3/TIM2_CH3[TIME3_CH1] SSR      | 21 PB1/AIN1[TIM1_CH2N] -
   10 PA4                     BUZZER   | 22 PB0/AIN0[TIM1_CH1N] -
   11 PA5                     -        | 23 PE7/AIN8            -
   12 PA6                     ISR      | 24 PE6/AIN9            -
   ------------------------------------|--------------------------------
   25 PE5/SPI_NSS          !CS_MAX6675 | 37 PE3/TIM1_BKIN       -
   26 PC1/TIM1_CH1/UART2_CK   CXAX     | 38 PE2/I2C_SDA         I2C_SDA
   27 PC2/TIM1_CH2            C6A5     | 39 PE1/I2C_SCL         I2C_SCL
   28 PC3/TIM1_CH3            C4A3     | 40 PE0/CLK_CC0         -
   29 PC4/TIM1_CH4            C2A1     | 41 PD0/TIM3_CH2...     -
   30 PC5/SPI_SCK             SPI_SCLK | 42 PD1/SWIM            SWIM
   31 VSSIO_2                          | 43 PD2/TIM3_CH1...     GFE 
   32 VDDIO_2                          | 44 PD3/TIM2_CH2...     FED
   33 PC6/SPI_MOSI            -        | 45 PD4/TIM2_CH1[BEEP]  EDC
   34 PC7/SPI_MISO            SPI_MISO | 46 PD5/UART2_TX        TX
   35 PG0                     -        | 47 PD6/UART2_RX        RX
   36 PG1                     -        | 48 PD7/TLI[TIM1_CH4]   CBA
   ---------------------------------------------------------------------

  Schematic of the bit numbers for the display LED's. Useful if custom characters are needed.
 
            * a * b   --------    *    --------       * C
                     /   a   /    g   /   a   /       e f
             d    f /       / b    f /       / b    ----
            ---     -------          -------     f / a / b
            *     /   g   /        /   g   /       ---
            c  e /       / c    e /       / c  e / g / c
                 --------    *    --------   *   ----  *
                   d         dp     d        dp   d    dp
  ==================================================================*/
#ifndef __W3230_MAIN_H__
#define __W3230_MAIN_H__

#include <iostm8s105c6.h>
#include <stdint.h>
#include "delay.h"
     
//-------------------------------------------------------------
// PORTG: PG1 and PG0 are not used
//-------------------------------------------------------------

//-------------------------------------------------------------
// PORTE: PE7,PE6,PE4 (not present), PE3 and PE0 are not used
//-------------------------------------------------------------
#define nCS      (0x20) /* PE5: CS line for MAX6675, active-low  */
#define I2C_SDA  (0x04) /* PE2: I2C sda line */
#define I2C_SCL  (0x02) /* PE1: I2C scl line */

#define MAX6675_CSb (PE_ODR_ODR5)

//-------------------------------------------------------------
// PORTD
//-------------------------------------------------------------
#define CBA      (0x80) /* PD7: Pin 4 of 7-segment display */
#define UART_RX  (0x40) /* PD6: Controlled by UART 2 */
#define UART_TX  (0x20) /* PD5: Controlled by UART 2 */
#define EDC      (0x10) /* PD4: Pin 6 of 7-segment display */
#define FED      (0x08) /* PD3: Pin 7 of 7-segment display */
#define GFE      (0x04) /* PD2: Pin 8 of 7-segment display */
#define SWIM     (0x02) /* PD1: Do not Initialize */
#define PGF      (0x01) /* PD0: Pin 9 of 7-segment display */
#define PD_SSD   (CBA | EDC | FED | GFE | PGF)

#define CBAb     (PD_ODR_ODR7)
#define EDCb     (PD_ODR_ODR4)
#define FEDb     (PD_ODR_ODR3)
#define GFEb     (PD_ODR_ODR2)
#define PGFb     (PD_ODR_ODR0)

//-------------------------------------------------------------
// PORTC: PC6 is not used, PC0 is not present
//        SPI lines are controlled by the SPI-device
//-------------------------------------------------------------
#define SPI_MISO (0x80) /* PC7: SPI MISO pin */
#define SPI_MOSI (0x40) /* PC6: SPI MOSI pin, not used */
#define SPI_CLK  (0x20) /* PC5: SPI clock line */
#define C2A1     (0x10) /* PC4: CC for display 2, CA for display 1 */
#define C4A3     (0x08) /* PC3: CC for display 4, CA for display 3 */
#define C6A5     (0x04) /* PC2: CC for display 6, CA for display 5 */
#define CXAX     (0x02) /* PC1: CC for remaining leds, CA for remaining leds */
#define PC_SSD   (C2A1 | C4A3 | C6A5 | CXAX) /* All CA and CC lines */

#define C2A1b    (PC_ODR_ODR4)
#define C4A3b    (PC_ODR_ODR3)
#define C6A5b    (PC_ODR_ODR2)
#define CXAXb    (PC_ODR_ODR1)

//-------------------------------------------------------------
// PORTB: PB1 and PB0 are not used
//-------------------------------------------------------------
#define KEY_UP     (0x80) /* PB7: UP button */
#define KEY_PWR    (0x40) /* PB6: PWR button */
#define KEY_DOWN   (0x20) /* PB5: DOWN button */
#define KEY_SET    (0x10) /* PB4: SET button */
#define LD_RED     (0x08) /* PB3: Red LED */
#define LD_BLUE    (0x04) /* PB2: Blue LED */
#define PB_KEYS    (KEY_UP | KEY_PWR | KEY_DOWN | KEY_SET)
#define PB_LEDS    (LD_RED | LD_BLUE)

#define LD_REDb    (PB_ODR_ODR3)
#define LD_BLUEb   (PB_ODR_ODR2)

//-------------------------------------------------------------
// PORTA: PA7 and PA0 not present, PA5, PA2 and PA1 not used 
//-------------------------------------------------------------
#define ISR_OUT  (0x40) /* PA6: ISR measurement */
#define BUZZER   (0x10) /* PA4: Buzzer */
#define SSR      (0x08) /* PA3: SSR output */

#define BUZZERb  (PA_ODR_ODR4)
#define SSRb     (PA_ODR_ODR3)

#define SCL_in    (PE_DDR &= ~I2C_SCL) 			    /* Set SCL to input */
#define SDA_in    (PE_DDR &= ~I2C_SDA) 			    /* Set SDA to input */
#define SCL_out   {PE_DDR |=  I2C_SCL; PE_CR1 |=  I2C_SCL;} /* Set SCL to push-pull output */
#define SDA_out   {PE_DDR |=  I2C_SDA; PE_CR1 |=  I2C_SDA;} /* Set SDA to push-pull output */
#define SCL_read  (PE_IDR &   I2C_SCL) 			    /* Read from SCL */
#define SDA_read  (PE_IDR &   I2C_SDA) 			    /* Read from SDA */
#define SCL_0     {PE_ODR &= ~I2C_SCL; i2c_delay_5usec(1);} /* Set SCL to 0 */
#define SCL_1     {PE_ODR |=  I2C_SCL; i2c_delay_5usec(1);} /* Set SCL to 1 */
#define SDA_1     (PE_ODR |=  I2C_SDA) 			    /* Set SDA to 1 */
#define SDA_0     (PE_ODR &= ~I2C_SDA) 			    /* Set SDA to 0 */

//--------------------------------
//  A   B   C   D   E   F   G   dp
//--------------------------------
#define LED_OFF	(0x00) 
#define LED_ON  (0xFF) 
#define LED_MIN (0x02) 
#define LED_DP  (0x01) 
#define LED_EQ  (0x12) 
#define LED_0	(0xFC) 
#define LED_1	(0x60) 
#define LED_2	(0xDA)  
#define LED_3  	(0xF2) 
#define LED_4  	(0x66) 
#define LED_5  	(0xB6)  
#define LED_6  	(0xBE) 
#define LED_7  	(0xE0) 
#define LED_8  	(0xFE) 
#define LED_9  	(0xF6)
#define LED_A  	(0xEE)
#define LED_a	(0xFA)
#define LED_b	(0x3E) 
#define LED_C	(0x9C)
#define LED_c	(0x1A)
#define LED_d	(0x7A)
#define LED_e	(0xDE) 
#define LED_E	(0x9E)
#define LED_F	(0x8E)
#define LED_H	(0x6E)
#define LED_h	(0x2E)
#define LED_I	(0x60) 
#define LED_J	(0x78) 
#define LED_L	(0x1C)
#define LED_n	(0x2A) 
#define LED_O	(0xFC)
#define LED_o	(0x3A)
#define LED_P	(0xCE) 
#define LED_r	(0x0A)	
#define LED_S	(0xB6)
#define LED_t	(0x1E)
#define LED_U	(0x7C)
#define LED_u	(0x38) 
#define LED_y	(0x76)

//----------------------------------
//  States for multiplexer function
//----------------------------------
#define STD_CA1 (0)  /* Display 1 CA */
#define STD_CA3	(1)  /* Display 3 CA */
#define STD_CA5	(2)  /* Display 5 CA */ 
#define STD_CAX	(3)  /* Display with remaining leds, CA */
#define STD_CC2 (4)  /* Display 2 CC */
#define STD_CC4	(5)  /* Display 4 CC */
#define STD_CC6	(6)  /* Display 6 CC */ 
#define STD_CCX	(7)  /* Display with remaining leds, CC */
                     
// Function prototypes
void setAsInputs(void);
void setAsOutputs(uint8_t maskC, uint8_t maskD);
void multiplexer(void);
void initialise_system_clock(void);
void setup_timer2(void);
void setup_gpio_ports(void);
void adc_task(void);
void pid_to_time(void);
void std_task(void);
void conv_fahrenheit(void);
void ctrl_task(void);
void prfl_task(void);

#endif // __W3230_MAIN_H__
