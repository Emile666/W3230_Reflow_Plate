/*==================================================================
  File Name: misc.c
  Author   : Emile
  ------------------------------------------------------------------
  Purpose  : This files contains the moving average filter and a
             slope-limiter function.
  ------------------------------------------------------------------
  This file is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  This software is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this software.  If not, see <http://www.gnu.org/licenses/>.
  ================================================================== */ 
#include "misc.h"

void init_moving_average(ma *p, uint8_t N, float init_val)
/*------------------------------------------------------------------
  Purpose  : This function initializes the Moving Average (MA) struct
             that is used in the moving_average() function
  Variables:
        *p : Pointer to the ma struct (specify a new struct for every
             new filter!)
         N : The order of the Moving Average filter (1 = No filtering)
   init_val: The value to init. the MA filter to
  Returns  : -
  ------------------------------------------------------------------*/
{
   uint8_t i; // temp. var.

   p->N     = N;        // order of MA filter
   p->index = 0;        // index in cyclic array
   p->sum   = init_val; // running sum
   for (i = 0; i < N; i++)
   {
      p->T[i] = init_val / N; // set cyclic array to init. value
   } // for
} // init_moving_average()

float moving_average(ma *p, float x)
/*------------------------------------------------------------------
  Purpose  : This function calculates the Moving Average (MA) of the
             input signal x. An MA filter of order N is a low-pass
             filter with a zero at fs / N (or 2.pi/N). It's Z transform
             function is:
                                 -1         -(N-1)
             Y[z] = X[z] . (1 + z  + ... + Z      )

             Initialization: p->N must have a value.
                             p->sum and p->index should be init. to 0.
  Variables:
        *p : Pointer to the ma struct (specify a new struct for every
             new filter!)
         x : The actual input value
  Returns  : Filter output value
  ------------------------------------------------------------------*/
{
   p->sum -= p->T[p->index];  // subtract value from running sum
   p->T[p->index] = x / p->N; // store new value in array
   p->sum += p->T[p->index];  // update running sum with new value
   if (++(p->index) >= p->N)  // update index in cyclic array
   {
      p->index = 0; // restore to 1st position
   } // if
   return p->sum;   // return value = filter output
} // moving_average()

void slope_limiter(const int16_t lim, const int16_t Told, int16_t *Tnew)
/*------------------------------------------------------------------
  Purpose  : This function limits the increase of Tnew by lim.

                              Tnew
                               |  ------------  lim
                               |/
              -----------------/------------------ -> Tin - Tout
                              /|
              -lim ----------  |

  Variables:
       lim : The limiting value
      Told : The previous value of the variable to limit
      Tnew : The output value of the variable to limit
  Returns  : none
  ------------------------------------------------------------------*/
{
   int16_t diff = *Tnew - Told; // calculate difference

   if      (diff > lim)  *Tnew =  Told + lim;
   else if (diff < -lim) *Tnew =  Told - lim;
   // else: do nothing, Tnew is within slope limits
} // slope_limiter()
