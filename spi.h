#ifndef _STM8_SPI_H
#define _STM8_SPI_H
/*==================================================================
   File Name: spi.h
   Author   : Emile
  ------------------------------------------------------------------
  Purpose : This files contains the SPI related functions 
            for the STM8 uC.
  ------------------------------------------------------------------
  This file is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  This file is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this file.  If not, see <http://www.gnu.org/licenses/>.
  ================================================================== */ 
#include <stdint.h>
    
void     spi_init(void); // Initializes the SPI Interface. Needs to be called only once
void     spi_write(uint8_t data);
uint8_t  spi_read(void);

#endif
