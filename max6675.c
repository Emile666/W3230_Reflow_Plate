/*==================================================================
  File Name    : max6675.c
  Author       : Emile
  ------------------------------------------------------------------
  Purpose : This files contains the routines for the MAX6675
            K-type thermocouple IC.
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
#include "max6675.h"
#include "spi.h"
#include "w3230_main.h"

bool no_tc  = false; // 1 = no thermocouple present
bool err_id = false; // 1 = Device ID bit is not 0

/*------------------------------------------------------------------
  Purpose  : This function writes 1 16-bit data-byte to the MAX7219.
  Variables: dat: the data-byte to write to the MAX7219
  Returns  : -
  ------------------------------------------------------------------*/
uint16_t max6675_read(void)
{
    uint8_t  bt;
    uint16_t ret;
    
    MAX6675_CSb = 0;         // Enable CS pin of MAX6675
    SPI_CR1_SPE = 1;         // Enable SPI
    
    spi_write(0x00);         // generate clock
    bt  = spi_read();        // read MSB
    ret = (uint16_t)bt << 8; // save MSB
    spi_write(0x00);         // generate clock
    bt = spi_read();         // read LSB
    ret |= bt;               // save LSB  
    err_id = (ret & 0x8002) ? true : false;
    no_tc  = (ret & 0x0004) ? true : false;
    ret &= 0x7FF8;           // clear non temperature bits
    ret >>= 3;               // temperature reading in bits 11-0
    
    while (!SPI_SR_TXE)  delay_usec(5); // wait until TX Register is empty
    while (SPI_SR_BSY)   delay_usec(5); // wait until SPI is not busy anymore
    SPI_DR;                 // dummy read;
    MAX6675_CSb = 1;        // Disable CS pin of MAX6675
    SPI_CR1_SPE = 0;        // Disable SPI
    return ret;             // return 12-bit temperature reading
} // max6675_read()

