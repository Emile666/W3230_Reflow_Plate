/*==================================================================
  File Name    : w3230_lib.c
  Author       : Mats Staffansson / Emile
  ------------------------------------------------------------------
  Purpose : This files contains the relevant functions for the menu,
            thermostat control and other functions needed for the
            custom build W3230 hardware.
  ------------------------------------------------------------------
  This is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  This file is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this file.  If not, see <http://www.gnu.org/licenses/>.
  ==================================================================
*/ 
#include <intrinsics.h>
#include "w3230_lib.h"
#include "pid.h"
#include "uart.h"
#include <stdio.h>

// LED character lookup table (0-9)
const uint8_t led_lookup[] = {LED_0,LED_1,LED_2,LED_3,LED_4,LED_5,LED_6,LED_7,LED_8,LED_9};

//----------------------------------------------------------------------------
// These values are stored directly into EEPROM
//----------------------------------------------------------------------------
__root __eeprom const int16_t eedata[] =
{
    // STM8S105C6 with 1024 bytes EEPROM
    25, 5,  25, 60,120,100,140, 20,170, 20,170,120, 25,  0,  0,  0,  0, 0, 0, // Pr0 Sn42-Bi58 138 �C (SP0, dh0, ..., dh8, SP9) 
    25, 5,  25, 60,150,120,150, 60,225, 20,225,120, 25,  0,  0,  0,  0, 0, 0, // Pr1 Lead (SP0, dh0, ..., dh8, SP9)
    25, 5,  25, 60,150,120,180, 60,255, 15,255,120, 25,  0,  0,  0,  0, 0, 0, // Pr2 Lead-Free (SP0, dh0, ..., dh8, SP9)
    25, 0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0, 0, // Pr3 (SP0, dh0, ..., dh8, SP9)
    25, 0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0, 0, // Pr4 (SP0, dh0, ..., dh8, SP9)
    25, 5,  25,  1,100,120,  1,150,120,  1,200,120,  1,250,120, 10, 25, 0, 0, // Pr5 Step-responses (SP0, dh0, ..., dh8, SP9)
   MENU_DATA(EEPROM_DEFAULTS) 
   1 // Last one, not used anymore
}; // eedata[]

// Global variables to hold LED data (for multiplexing purposes)
uint8_t top_10, top_1, top_01;  // values of 10s, 1s and 0.1s
uint8_t bot_10, bot_1, bot_01;  // values of 10s, 1s and 0.1s

uint8_t  menustate     = MENU_IDLE; // Current STD state number for menu_fsm()
uint8_t  ret_state;                 // menustate to return to
bool     menu_is_idle  = true;  // No menu active within STD
bool     fahrenheit    = false; // false = Celsius, true = Fahrenheit
uint8_t  menu_item     = 6;     // Current menu-item: [0..NO_OF_PROFILES]
uint8_t  config_item   = 0;     // Current index within profile or parameter menu
uint8_t  m_countdown   = 0;     // Timer used within menu_fsm()
uint8_t  _buttons      = 0;     // Current and previous value of button states
int16_t  config_value;          // Current value of menu-item
int8_t   key_held_tmr;          // Timer for value change acceleration
uint8_t  sensor2_selected = 0;  // DOWN button pressed < 3 sec. shows 2nd temperature / pid_output
int16_t  setpoint;              // Setpoint variable
uint16_t curr_dur = 0;          // Duration of a temperature within a profile
uint8_t  curr_step = 0;         // Current step number within a profile
int16_t  pid_out  = 0;          // Output from PID controller in %
bool     pid_ena = true;        // True = PID controller enabled
bool     run_profile = false;   // True = start Profile run
uint8_t  std_tc = STD_OFF;      // State for Temperature Control

// External variables, defined in other files
extern int16_t  temp_tc;        // Thermocouple temperature in �C
extern int16_t  temp_tf;        // Thermocouple temperature in �F
extern int16_t  kc;             // Parameter value for Kc value in %/�C
extern int16_t  ti;             // Parameter value for I action in seconds
extern int16_t  td;             // Parameter value for D action in seconds
extern uint8_t  ts;             // Parameter value for sample time [sec.]
extern char     version[];      // Version info string
extern int32_t  kpi, kii, kdi;
extern bool     bz_on;          // true = buzzer-on
extern uint8_t  bz_rpt_max;     // number of buzzer repeats

// This contains the definition of the menu-items for the parameters menu
const struct s_menu menu[] = 
{
    MENU_DATA(TO_STRUCT)
}; // menu[]

/*-----------------------------------------------------------------------------
  Purpose  : This routine does a divide by 10 using only shifts
  Variables: n: the number to divide by 10
  Returns  : the result
  ---------------------------------------------------------------------------*/
uint16_t divu10(uint16_t n) 
{
  uint16_t q, r;

  q = (n >> 1) + (n >> 2);       // q = 1/2 + 1/4 = 3/4
  q = q + (q >> 4);              // q = 3/4 + 3/64 = 51/64
  q = q + (q >> 8);              // q = 51/64 + 51/(16384) = 13107/16384
  q = q >> 3;                    // q = 13107 / 131072
  r = n - ((q << 3) + (q << 1)); // r = 1 - (13107/16384 + 13107/65536) = 1/65536
  return q + ((r + 6) >> 4);     // 13107/131072 + 1/1048576 = 104857 / 1048576  
} // divu10()

/*-----------------------------------------------------------------------------
  Purpose  : This routine is called by menu_fsm() to show the name of the
             menu-item. This can be either one of the Profiles (Pr0, Pr1, ...),
             the text 'SET' (when in the parameter menu) of the text 'th' when
             in thermostat mode.
  Variables: run_mode: [0..NR_OF_PROFILES]
  Returns  : -
  ---------------------------------------------------------------------------*/
void prx_to_led(uint8_t run_mode, uint8_t is_menu)
{
    if (run_mode < NO_OF_PROFILES)
    {   // one of the profiles
	bot_10 = LED_P;
	bot_1  = LED_r;
	bot_01 = led_lookup[run_mode];
    } // if
    else 
    { // parameter menu
	if (is_menu)
        {   // within menu
            bot_10 = LED_S;
            bot_1  = LED_E;
            bot_01 = LED_t;
        } // if
        else if (ts == 0)
        {   // Thermostat Mode
            bot_10 = LED_t; 
            bot_1  = LED_h;
            bot_01 = LED_OFF;
        } // else if
        else 
        {   // PID controller mode
            bot_10 = LED_P;
            bot_1  = LED_I;
            bot_01 = LED_d;
        } // else
    } // else
} // prx_to_led()

/*-----------------------------------------------------------------------------
  Purpose  : This routine converts the number in value to a BCD digit.
             Which digit to convert is specified by digit (1000, 100, 10).
  Variables: *value  : the value to convert into a BCD code
             digit   : 1000, 100, 10, 1
             *led    : the 7-segment display variable
             lz      : 1 = leading zero, 0 = display off
  Returns  : -
  ---------------------------------------------------------------------------*/
void val_to_bcd(int16_t *value, uint16_t digit, uint8_t *led, uint8_t lz)
{
    uint8_t i;
    
    if (*value >= digit)
    {
        for(i = 0; *value >= digit; i++)
        {
            *value -= digit;
        } // for
        *led = led_lookup[i & 0x0f];
    } // if
    else if (lz)
         *led = LED_0;   // Leading zero
    else *led = LED_OFF; // display off
} // val_to_bcd()

/*-----------------------------------------------------------------------------
  Purpose  : This routine is called by menu_fsm() to show the value of a
             temperature or a non-temperature value.
             In case of a temperature, a decimal point is displayed (for 0.1).
             In case of a non-temperature value, only the value itself is shown.
  Variables: value  : the value to display
             decimal: 0=display as integer, 1=display temperature as xxx.1
             row    : ROW_TOP = display on top row of 7-segment displays
                      ROW_BOT = display on bottom row of 7-segment displays
  Returns  : -
  ---------------------------------------------------------------------------*/
void value_to_led(int16_t value, uint8_t decimal, uint8_t row) 
{
    int16_t val = value; // copy of value
    int16_t val2;        // copy of val
    uint8_t *p10, *p1, *p01; // pointers to 7-segment display values
    
    if (val < 0) 
    {   // Handle negative values
        val  = -val;
        if (val >= 100) 
        {
          val = divu10(val);  // loose the decimal
          decimal = DECIMAL0; // no decimal point 
        } // if
    } // if
    else if (val >= 1000)
    {
        val = divu10(val); // loose the decimal
        decimal = DECIMAL0;
    } // else if
    val2 = val;
    
    if (row == ROW_TOP)
    {
        p10  = &top_10;
        p1   = &top_1;   
        p01  = &top_01;
    }
    else
    {   // row == ROW_BOT
        p10  = &bot_10;
        p1   = &bot_1;   
        p01  = &bot_01;
    } // else
	// Convert value to BCD and set LED outputs
	val_to_bcd(&val,  100, p10 ,0);
	val_to_bcd(&val,   10, p1  ,(*p10  != LED_OFF));
	val_to_bcd(&val,    1, p01 ,1);

    if (decimal == DECIMAL1)
    {  // this is a temperature
       if (*p1 == LED_OFF) *p1 = LED_0; // add leading zero if needed
	   *p1 |= LED_DP;                   // add decimal point
    } // if

    if (value < 0)
    {   // original value < 0 
        if ((val2 < 10) && (decimal == DECIMAL0)) 
                              *p1   = LED_MIN;
        else if (val2 < 100)  *p10  = LED_MIN;
        //else if (val2 < 1000) *p100 = LED_MIN;
        // else value >= 1000: prevented by divu10()
    } // if
} // value_to_led()

/*-----------------------------------------------------------------------------
  Purpose  : This task updates the current running profile. A profile consists
             of several temperature-time pairs. When a time-out occurs, the
             next temperature-time pair within that profile is selected.
             Updates are stored in the EEPROM configuration.
             It is called every second
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void update_profile(void)
{
  uint8_t  profile_no = eeprom_read_config(EEADR_MENU_ITEM(rn));
  uint8_t  profile_step_eeaddr;  // Address index in eeprom for step nr in profile
  uint16_t profile_step_dur;     // Duration of current step
  int16_t  profile_next_step_sp; // Setpoint value of next step in profile
  int16_t  profile_step_sp;      // Setpoint value of current step in profile
  uint16_t t;
  int32_t  sp;
  uint8_t  i;
  
  // Running profile?
  if ((profile_no < THERMOSTAT_MODE) && run_profile)
  {
    // Sanity check
    if(curr_step > NO_OF_TT_PAIRS-1) curr_step = NO_OF_TT_PAIRS - 1;
    
    // Read SP, duration (sec.) and next SP from Profile in EEPROM
    profile_step_eeaddr  = EEADR_PROFILE_SETPOINT(profile_no, curr_step);
    profile_step_dur     = eeprom_read_config(profile_step_eeaddr + 1);
    profile_next_step_sp = eeprom_read_config(profile_step_eeaddr + 2);
    
    // Reached end of step?
    if (++curr_dur >= profile_step_dur) 
    {   // Update setpoint with value from next step
      setpoint = profile_next_step_sp;
      // Is this the last step (next step is number 9 or next step duration is 0)?
      if ((curr_step == NO_OF_TT_PAIRS-1) || eeprom_read_config(profile_step_eeaddr + 3) == 0) 
      {   
        run_profile = false; // Finished running with profile
      } // if
      curr_dur = 0; // Reset duration
      curr_step++;  // Update step
    } // if
    else
    {
      // Calculate next setpoint with ramping
      profile_step_sp = eeprom_read_config(profile_step_eeaddr);
      t  = curr_dur << 6;
      sp = 32;
      for (i = 0; i < 64; i++) 
      {   // Linear interpolation of new setpoint (64 substeps)
        if (t >= profile_step_dur) 
        {
          t  -= profile_step_dur;
          sp += profile_next_step_sp;
        } // if
        else 
        {
          sp += profile_step_sp;
        } // else
      } // for
      sp >>= 6;
      // Update setpoint
      setpoint = sp;
    } // else
  } // if
  else 
  { // thermostat mode
    curr_dur  = 0;
    curr_step = 0;
    setpoint  = eeprom_read_config(EEADR_MENU_ITEM(SP));
  } // else
} // update_profile()

/*-----------------------------------------------------------------------------
  Purpose  : This routine checks if a value is within a minimum and maximul value.
             If the value is larger than the maximum, the minimum value is 
             returned (roll-over). If the value is smaller than the minimum, 
             the maximum value is returned (roll-over).
  Variables: x  : the value to check
             min: the minimum allowed value         
             max: the maximum allowed value         
  Returns  : the value itself, or the roll-over value in case of a max./min.
  ---------------------------------------------------------------------------*/
int16_t range(int16_t x, int16_t min, int16_t max)
{
    if (x > max) return min;
    if (x < min) return max;
    return x;
} // range()

/*-----------------------------------------------------------------------------
  Purpose  : This routine checks a parameter value and constrains it to a 
             maximum/minimum value.
  Variables: config_value : the value to check for
             eeadr        : the number of a 16-bit variable within the EEPROM.         
  Returns  : the value itself, or the roll-over value in case of a max./min.
  ---------------------------------------------------------------------------*/
int16_t check_config_value(int16_t config_value, uint8_t eeadr)
{
    int16_t t_min = 0, t_max = 999;
    uint8_t type;
    
    if (eeadr < EEADR_MENU)
    {   // One of the Profiles
        while (eeadr >= PROFILE_SIZE)
        {   // Find the eeprom address within a profile
            eeadr -= PROFILE_SIZE;
        } // while
        if (!(eeadr & 0x1))
        {   // Only constrain a temperature
            t_min = (fahrenheit ? TEMP_MIN_F : TEMP_MIN_C);
            t_max = (fahrenheit ? TEMP_MAX_F : TEMP_MAX_C);
        } // if
    } else { // Parameter menu
        type = menu[eeadr - EEADR_MENU].type;
        if (type == t_temperature)
        {
            t_min = (fahrenheit ? TEMP_MIN_F : TEMP_MIN_C);
            t_max = (fahrenheit ? TEMP_MAX_F : TEMP_MAX_C);
        } else if (type == t_tempdiff)
        {   // the temperature correction variables
            t_min = (fahrenheit ? TEMP_CORR_MIN_F : TEMP_CORR_MIN_C);
            t_max = (fahrenheit ? TEMP_CORR_MAX_F : TEMP_CORR_MAX_C);
        } else if (type == t_parameter)
        {
            t_max = 999;
        } else if (type == t_boolean)
        {   // the control variables
            t_max = 1;
        } else if (type == t_filter)
        {   // Order for MA-filter
            t_max = 5;
        } else if (type == t_percent)
        {   // Percentage
            t_max = 100;
        } else if (type == t_runmode)
        {
            t_max = NO_OF_PROFILES;
        } // else if
    } // else
    return range(config_value, t_min, t_max);
} // check_config_value()

/*-----------------------------------------------------------------------------
  Purpose  : This routine reads the values of the buttons and returns the
             result. Routine should be called every 100 msec.
             The result is returned in the global variable _buttons. 
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void read_buttons(void)
{
    uint8_t b;
    
    // Save registers that interferes with LED's and disable common-cathodes
    _buttons <<= 4;            // make room for new values of buttons
    b          = (PB_IDR & PB_KEYS) >> 4;   // KEYS in bits 3..0
    b          = (b ^ 0x0F) & 0x0F;         // Invert buttons (0 = pressed)
    _buttons  |= b;            // add buttons
} // read_buttons()

/*-----------------------------------------------------------------------------
  Purpose  : This routine is the Finite State Machine (FSM) that controls the
             menu for the 7-segment displays and should be called every 100 msec.
             It used a couple of global variables.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void menu_fsm(void)
{
   uint8_t run_mode, adr, type, eeadr_sp;
   
   if (m_countdown) m_countdown--; // countdown counter
    
   switch (menustate)
   {
       //--------------------------------------------------------------------         
       case MENU_IDLE:
            if (BTN_PRESSED(BTN_PWR))
            {
                m_countdown = TMR_REFLOW_START;
                menustate   = MENU_REFLOW_WAIT;
            } else if (_buttons)
            {
                if (BTN_PRESSED(BTN_UP))
                {   // UP button pressed: show firmware version nr
                    menustate = MENU_SHOW_VERSION;
                } else if (BTN_PRESSED(BTN_DOWN))
                {   // DOWN button pressed: show current profile details
                    m_countdown = TMR_SHOW_PROFILE_ITEM;
                    menustate   = MENU_SHOW_STATE_DOWN;
                } else if (BTN_RELEASED(BTN_SET))
                {   // SET button pressed
                    menustate = MENU_SHOW_MENU_ITEM;
                } // else if
            } // else
	     break;
       //--------------------------------------------------------------------         
       case MENU_REFLOW_WAIT:
            if (m_countdown == 0)
            {
                run_profile  = !run_profile; // start/stop running a profile
                if (run_profile)
                { // Set Feed-Forward factor for PID-controller at start
                  pid_out = eeprom_read_config(EEADR_MENU_ITEM(FFF));
                } // if
                menustate    = MENU_IDLE;    // return to IDLE state
                bz_rpt_max   = 1;            // Sound buzzer once
                bz_on        = true;         // Enable buzzer in ISR
            } else if(!BTN_HELD(BTN_PWR))
            {   // 0 = temp_tc, 1 = pid-output
                if (++sensor2_selected > 1)  
                   sensor2_selected = 0;
                menustate = MENU_IDLE;
            } // else if
            break; // MENU_REFLOW_WAIT
       //--------------------------------------------------------------------         
       case MENU_SHOW_VERSION: // Show firmware version number
            top_10 = LED_u; top_1 = LED_E; top_01 = LED_r;
            bot_10 = led_lookup[(uint8_t)(version[14] - '0') & 0x0F] | LED_DP;
            bot_1  = led_lookup[(uint8_t)(version[16] - '0') & 0x0F];
            bot_01 = led_lookup[(uint8_t)(version[17] - '0') & 0x0F];
	    if(!BTN_HELD(BTN_UP)) menustate = MENU_IDLE;
	    break;
       //--------------------------------------------------------------------         
       case MENU_SHOW_STATE_DOWN: // Show Profile-number
	    run_mode = eeprom_read_config(EEADR_MENU_ITEM(rn));
            top_10 = LED_r; top_1 = LED_u; top_01 = LED_n;
            prx_to_led(run_mode, LEDS_RUN_MODE);
            if ((run_mode < THERMOSTAT_MODE) && (m_countdown == 0))
            {
                m_countdown = TMR_SHOW_PROFILE_ITEM;
                menustate   = MENU_SHOW_STATE_DOWN_2;
            } // if
	    if(!BTN_HELD(BTN_DOWN)) menustate = MENU_IDLE;
	    break;
       //--------------------------------------------------------------------         
       case MENU_SHOW_STATE_DOWN_2: // Show current step number within profile
	    top_10  = LED_S; top_1 = LED_t; 
            top_01 = LED_OFF;
            value_to_led(curr_step,DECIMAL0, ROW_BOT);
            if (m_countdown == 0)
            {
                m_countdown = TMR_SHOW_PROFILE_ITEM;
                menustate   = MENU_SHOW_STATE_DOWN_3;
	    }
	    if(!BTN_HELD(BTN_DOWN)) menustate = MENU_IDLE;
	    break;
       //--------------------------------------------------------------------         
       case MENU_SHOW_STATE_DOWN_3: // Show current duration of running profile
            top_10 = LED_d; top_1 = LED_h; 
            top_01 = LED_OFF;
            value_to_led(curr_dur,DECIMAL0,ROW_BOT);
            if(m_countdown == 0)
            {   // Time-Out
                m_countdown = TMR_SHOW_PROFILE_ITEM;
                menustate   = MENU_SHOW_STATE_DOWN;
            } // if
            if(!BTN_HELD(BTN_DOWN))
            {   // Down button is released again
                menustate = MENU_IDLE;
            } // if
            break; // MENU_SHOW_STATE_DOWN_3
       //--------------------------------------------------------------------         
       case MENU_SHOW_MENU_ITEM: // S-button was pressed
            top_10 = LED_S; top_1 = LED_E; top_01 = LED_t;
            if (menu_item < NO_OF_PROFILES)
            {   // one of the profiles
                bot_10 = LED_P;
                bot_1  = LED_r;
                bot_01 = led_lookup[menu_item];
            } // if
            else 
            {   // Set Parameters menu
                bot_10 = LED_P; 
                bot_1  = LED_A;
                bot_01 = LED_r;
            } // else
            m_countdown = TMR_NO_KEY_TIMEOUT;
            menustate   = MENU_SET_MENU_ITEM;
            break; // MENU_SHOW_MENU_ITEM
       //--------------------------------------------------------------------         
       case MENU_SET_MENU_ITEM:
            if(m_countdown == 0 || BTN_RELEASED(BTN_PWR))
            {   // On Time-out of S-button released, go back
                menustate = MENU_IDLE;
            } else if(BTN_RELEASED(BTN_UP))
            {
                if(++menu_item > MENU_ITEM_NO) menu_item = 0;
                menustate = MENU_SHOW_MENU_ITEM;
            } else if(BTN_RELEASED(BTN_DOWN))
            {
                if(--menu_item > MENU_ITEM_NO) menu_item = MENU_ITEM_NO;
                menustate = MENU_SHOW_MENU_ITEM;
            } else if(BTN_RELEASED(BTN_SET))
            {   // only go to next state if S-button is released
                config_item = 0;
                menustate   = MENU_SHOW_CONFIG_ITEM;
            } // else if
            break; // MENU_SET_MENU_ITEM
       //--------------------------------------------------------------------         
       case MENU_SHOW_CONFIG_ITEM: // S-button is released
            if (menu_item < MENU_ITEM_NO)
            {
                if(config_item & 0x1) 
                {   
                    top_10 = LED_d; // duration: 2nd value of a profile-step
                    top_1  = LED_h;
                } else {
                    top_10 = LED_S; // setpoint: 1st value of a profile-step
                    top_1  = LED_P;
                } // else
                top_01 = led_lookup[(config_item >> 1)];
	    } else /* if (menu_item == 6) */
            {   // show parameter name
                top_10 = menu[config_item].led_c_10;
                top_1  = menu[config_item].led_c_1;
                top_01 = menu[config_item].led_c_01;
	    } // else
            adr          = MI_CI_TO_EEADR(menu_item, config_item);
            config_value = eeprom_read_config(adr);
            config_value = check_config_value(config_value, adr);
            m_countdown  = TMR_NO_KEY_TIMEOUT;
            ret_state    = MENU_SET_CONFIG_ITEM;   // return state
            menustate    = MENU_SHOW_CONFIG_VALUE; // display config value
	    break;
       //--------------------------------------------------------------------         
       case MENU_SET_CONFIG_ITEM:
	    if (m_countdown == 0)
            {   // Timeout, go back to idle state
                menustate = MENU_IDLE;
	    } else if(BTN_RELEASED(BTN_PWR))
            {   // Go back
                menustate = MENU_SHOW_MENU_ITEM;
            } else if(BTN_RELEASED(BTN_UP))
            {
                config_item++;
                if(menu_item < MENU_ITEM_NO)
                {
                    if(config_item >= PROFILE_SIZE)
                    {
                        config_item = 0;
                    } // if
                } else {
                    if(config_item >= MENU_SIZE)
                    {
                        config_item = 0;
                    }
                } // else
                ret_state = MENU_SHOW_CONFIG_ITEM;  // return state
                menustate = MENU_SHOW_CONFIG_VALUE; // display config value
            } else if(BTN_RELEASED(BTN_DOWN))
            {
                config_item--;
                if(menu_item < MENU_ITEM_NO)
                {   // One of the profiles
                    if(config_item >= PROFILE_SIZE)
                    {
                        config_item = PROFILE_SIZE-1;
                    } // if
                } else { // Menu with parameters
                    if(config_item > MENU_SIZE-1)
                    {
                        config_item = MENU_SIZE-1;
                    } // if
                } // else
                ret_state = MENU_SHOW_CONFIG_ITEM;  // return to this state
                menustate = MENU_SHOW_CONFIG_VALUE; // display config value
            } else if(BTN_RELEASED(BTN_SET))
            {   // S-button is released again
                m_countdown  = TMR_NO_KEY_TIMEOUT;
                menustate    = MENU_SET_CONFIG_VALUE; // display config value
            } // else if
            adr          = MI_CI_TO_EEADR(menu_item, config_item);
            config_value = eeprom_read_config(adr);
            config_value = check_config_value(config_value, adr);
            break; // MENU_SET_CONFIG_ITEM
       //--------------------------------------------------------------------         
       case MENU_SHOW_CONFIG_VALUE:
            if(menu_item < MENU_ITEM_NO)
            {   // Display duration and temperature
                value_to_led(config_value, DECIMAL0, ROW_BOT);
            } else 
            {   // menu_item == MENU_ITEM_NO
                type = menu[config_item].type;
                if(type == t_boolean)
                {   // Celsius or Fahrenheit
                    bot_10 = bot_1 = LED_OFF;
                    bot_01 = config_value ? LED_F : LED_C;
                } else if (type == t_runmode)
                {
                    prx_to_led(config_value, LEDS_RUN_MODE);
                } else { // others, display as integer
                    value_to_led(config_value,DECIMAL0, ROW_BOT);
                } // else
            } // else
            m_countdown  = TMR_NO_KEY_TIMEOUT;
            menustate    = ret_state; // return to indicated state
            break;
       //--------------------------------------------------------------------         
       case MENU_SET_CONFIG_VALUE:
            adr = MI_CI_TO_EEADR(menu_item, config_item);
            if (m_countdown == 0)
            {
                menustate = MENU_IDLE;
            } else if (BTN_RELEASED(BTN_PWR))
            {
                menustate = MENU_SHOW_CONFIG_ITEM;
            } else if(BTN_HELD_OR_RELEASED(BTN_UP)) 
            {
                config_value++;
                if ((config_value > 1000) || (--key_held_tmr < 0))
                {
                    config_value += 9;
                } // if
                /* Jump to exit code shared with BTN_DOWN case */
                goto chk_cfg_acc_label;
            } else if(BTN_HELD_OR_RELEASED(BTN_DOWN)) 
            {
                config_value--;
                if ((config_value > 1000) || (--key_held_tmr < 0))
                {
                    config_value -= 9;
                } // if
            chk_cfg_acc_label: // label for goto
                config_value = check_config_value(config_value, adr);
                ret_state    = MENU_SET_CONFIG_VALUE;  // return to this state
                menustate    = MENU_SHOW_CONFIG_VALUE; // show config_value
            } else if(BTN_RELEASED(BTN_SET))
            {
                if(menu_item == MENU_ITEM_NO)
                {   // We are in the parameter menu
                    if(config_item == rn)
                    {   // When setting run-mode, clear current step & duration
                        curr_step = 0;
                        curr_dur =  0;
                        if(config_value < THERMOSTAT_MODE)
                        {
                            eeadr_sp = EEADR_PROFILE_SETPOINT(((uint8_t)config_value), 0);
                            // Set initial value for SP
                            setpoint = eeprom_read_config(eeadr_sp);
                            // Hack in case inital step duration is '0'
                            if (eeprom_read_config(eeadr_sp+1) == 0)
                            {   // Set to thermostat mode
                                config_value = THERMOSTAT_MODE;
                            } // if
                        } // if
                    } // if
                } // if
                eeprom_write_config(adr, config_value);
                menustate = MENU_SHOW_CONFIG_ITEM;
            } else 
            {   // reset timer to default value
                key_held_tmr = TMR_KEY_ACC; 
            } // else
            break; // MENU_SET_CONFIG_VALUE
       //--------------------------------------------------------------------         
       default:
            menustate = MENU_IDLE;
            break;
   } /* switch(menustate) */
   menu_is_idle = (menustate == MENU_IDLE); // needed for ctrl_task()
} // button_menu_fsm()

/*-----------------------------------------------------------------------------
  Purpose  : This routine controls the PID controller. It should be 
             called once every second by ctrl_task() as long as TS is not 0. 
             The PID controller itself is called every TS seconds and only
             affects heating. 
             It runs in parallel with the thermostat relay control. So heating
             can be controlled either with the relays or with the SSR / pid-output.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void pid_control(int16_t temp)
{
    static uint8_t pid_tmr = 0;
    
    if (kc != eeprom_read_config(EEADR_MENU_ITEM(Hc)) ||
        ti != eeprom_read_config(EEADR_MENU_ITEM(Ti)) ||
        td != eeprom_read_config(EEADR_MENU_ITEM(Td)))
    {   // One or more PID parameters have changed
        kc = eeprom_read_config(EEADR_MENU_ITEM(Hc));
        ti = eeprom_read_config(EEADR_MENU_ITEM(Ti));
        td = eeprom_read_config(EEADR_MENU_ITEM(Td));
        init_pid(kc,ti,td,ts,temp); // Init PID controller
    } // if
    if (++pid_tmr >= ts) 
    {   // Call PID controller every TS seconds
        pid_ctrl(temp,&pid_out,setpoint,100,pid_ena);
        pid_tmr = 0;
    } // if
} // pid_control()
