/*==================================================================
  File name    : pid.c
  Author       : Emile
  ------------------------------------------------------------------
  Purpose : This file contains the main body of the PID controller.

           For design details, please check:
           document: PID_Controller_Calculus.pdf
           website : http://www.vandelogt.nl/uk_regelen_pid.php

            With the GUI, the following parameters can be changed:
            Kc: The controller gain in %/°C
            Ti: Time-constant for the Integral Gain in seconds
            Td: Time-constant for the Derivative Gain in seconds
            Ts: The sample period in seconds
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
#include "pid.h"

int16_t kc = 0;   // Parameter value for Kc value in %/°C
int16_t ti = 0;   // Parameter value for I action in seconds
int16_t td = 0;   // Parameter value for D action in seconds
float kpi;        // Internal P-action result for debugging
float kii;        // Internal I-action result for debugging
float kdi;        // Internal D-action result for debugging

// Init ts to 0 to disable pid-control and enable thermostat control
uint8_t  ts = 0;   // Parameter value for sample time [sec.]
float    ki;       // Internal value for I action
float    kd;       // Internal value for D action
int32_t  pp;       // debug
int16_t  yk_1;     // y[k-1]
int16_t  yk_2;     // y[k-2]
float    kif;
float    kdf;
float    ppf;

void init_pid(uint16_t kc, uint16_t ti, uint16_t td, uint8_t ts, uint16_t yk)
/*------------------------------------------------------------------
  Purpose  : This function initialises the Takahashi Type C PID
             controller.
  Variables: kc: Kc parameter value in %/°C    ; controls P-action
             ti: Ti parameter value in seconds ; controls I-action
             td: Td parameter value in seconds ; controls D-action
             ts: Ts parameter sample-time of pid-controller in seconds
             yk: actual temperature value

                   Kc.Ts
             ki =  -----   (for I-term)
                    Ti

                       Td
             kd = Kc . --  (for D-term)
                       Ts

  Returns  : No values are returned
  ------------------------------------------------------------------*/
{
   if (ti == 0) 
   {
       ki  = 0.0;
   } // if
   else 
   {
       ki = (float)kc * ts / ti;
   } // else
   if (ts == 0) 
   {
       kd  = 0.0;
   } // if
   else
   {
       kd = (float)kc * td / ts;
   } // else
   yk_2 = yk_1 = yk; // init. previous samples to current temperature
} // init_pid()

void pid_ctrl(int16_t yk, int16_t *uk, int16_t tset, int16_t lim, bool ena)
/*------------------------------------------------------------------
  Purpose  : This function implements the Takahashi Type C PID
             controller: the P and D term are no longer dependent
             on the setpoint, only on PV.
             This function should be called once every TS seconds.
  Variables:
        yk : The input variable y[k] (= measured temperature in °C)
       *uk : The pid-output variable u[k] [0%..+100%]
      tset : The setpoint value w[k] for the temperature in °C
      lim  : Upper-limit for *uk in %
  Returns  : No values are returned
  ------------------------------------------------------------------*/
{
    //-----------------------------------------------------------------------------
    // Takahashi Type C PID controller:
    //
    // e[k] = w[k] - y[k]
    //                                    Kc.Ts        Kc.Td
    // u[k] = u[k-1] + Kc.(y[k-1]-y[k]) + -----.e[k] + -----.(2.y[k-1]-y[k]-y[k-2])
    //                                      Ti           Ts
    //
    //-----------------------------------------------------------------------------
    if (ena)
    {
        kpi  = (float)kc * (yk_1 - yk);        // Kc.(y[k-1]-y[k])
        kii  = ki * (tset - yk);               // (Kc.Ts/Ti).e[k]
        kdi  = kd * ((yk_1 << 1) - yk - yk_2); // (Kc.Td/Ts).(2.y[k-1]-y[k]-y[k-2])
        *uk += (int16_t)(kpi + kii + kdi);
        // limit u[k] to lim and 0
        if (*uk > lim)    *uk = lim;
        else if (*uk < 0) *uk = 0;
    } // if
    else *uk = 0;
    
    yk_2  = yk_1; // y[k-2] = y[k-1]
    yk_1  = yk;   // y[k-1] = y[k]
} // pid_ctrl()

