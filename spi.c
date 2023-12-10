/*==================================================================
  File Name    : spi.c
  Author       : Emile
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
#include "spi.h"
#include "delay.h"
#include "w3230_main.h"

/*-----------------------------------------------------------------------------
  Purpose  : This function initializes the SPI bus controller
  Variables: --
  Returns  : --
  ---------------------------------------------------------------------------*/
void spi_init(void)
{
    SPI_CR1_SPE = 0;     // Disable SPI
    SPI_CR1     = 0x14;  // MSB first, disable SPI, 3 MHz clock, Master mode, SPI mode 0
    SPI_CR2     = 0x03;  // Select SW Slave Management and Master mode
    SPI_ICR     = 0x00;  // Disable SPI interrupt on error
    // SPI_NSS is already set to Output and a high-level by GPIO init.
} // spi_init()

/*-----------------------------------------------------------------------------
  Purpose  : This function sends one byte to the SPI device
  Variables: data: byte to be transferred
  Returns  : -
  ---------------------------------------------------------------------------*/
void spi_write(uint8_t data)
{   // At 3 Mbps, writing a byte takes approx. 2.5 usec.
    while (!SPI_SR_TXE) delay_usec(5); // wait until TX Register is empty
    SPI_DR = data; // send byte over SPI bus
    while (!SPI_SR_TXE) delay_usec(5); // wait until TX Register is empty
    while (SPI_SR_BSY)  delay_usec(5); // wait until SPI is not busy anymore
} // spi_write()

/*-----------------------------------------------------------------------------
  Purpose  : This function reads one byte from the SPI device
  Variables: --
  Returns  : byte read from the SPI device
  ---------------------------------------------------------------------------*/
uint8_t spi_read(void)
{   
    uint8_t reg;
    
    while (!SPI_SR_RXNE) delay_usec(5); // wait until RX Register is full
    reg = SPI_DR;
    SPI_SR; // clear possible OVR flag
    return reg;
} // spi_read()



