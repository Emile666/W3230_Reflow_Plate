/*==================================================================
  File Name    : max6675.h
  Author       : Emile
  ------------------------------------------------------------------
  Purpose : This file is the header-file for max6675.c and contains
            the routines to write to the MAX6675.
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
  along with this file. If not, see <http://www.gnu.org/licenses/>.
  ================================================================== */
#ifndef _MAX6675_H_
#define _MAX6675_H_
#include <stdint.h>
#include <stdbool.h>

uint16_t max6675_read(void);

#endif
