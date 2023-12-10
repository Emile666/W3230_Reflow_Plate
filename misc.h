#ifndef _MISC_H
#define _MISC_H
/*==================================================================
  File Name: misc.h
  Author   : Emile
  ------------------------------------------------------------------
  Purpose  : This is the header-file for misc.c
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
#include <stdint.h>

#define MAX_MA (5)
typedef struct _ma
{
	float   T[MAX_MA]; // array with delayed values of input signal
	uint8_t index;     // index in T[] where to store the new input value
	uint8_t N;         // The order of the MA filter. Note that N < MAX_MA
	float   sum;       // The running sum of the MA filter
} ma;

void  init_moving_average(ma *p, uint8_t N, float init_val);
float moving_average(ma *p, float x);
void  slope_limiter(const int16_t lim, const int16_t Told, int16_t *Tnew);

#endif