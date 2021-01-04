/*!
 *****************************************************************************
  @file:    ArduinoPort.cpp
  @author:  D. Bill (adapted from Analog Devices example code)
  @brief:   The port for Arduino/AdaFruit/SparkFun/... boards (based on ADICUP3029Port.c for ADI's ADICUP3029 board)

-----------------------------------------------------------------------------

@license: https://github.com/analogdevicesinc/ad5940lib/blob/master/LICENSE (accessed on December 3, 2020)

Copyright (c) 2019 Analog Devices, Inc.  All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
- Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.  
- Modified versions of the software must be conspicuously marked as such.
- This software is licensed solely and exclusively for use with processors/products manufactured by or for Analog Devices, Inc.
- This software may not be combined or merged with other code in any manner that would cause the software to become subject to terms and conditions which differ from those listed here.
- Neither the name of Analog Devices, Inc. nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
- The use of this software may or may not infringe the patent rights of one or more patent holders.  This license does not release you from the requirement that you obtain separate licenses from these patent holders to use this software.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF CLAIMS OF INTELLECTUAL PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

2019-01-10-7CBSD SLA

This file is a MODIFIED VERSION of the source code provided from Analog Devices!

*****************************************************************************/

#include "ad5940.h"
#include <SPI.h>

//******************which board?*************************
#define ADAFRUIT_FEATHER_M0_WIFI

//Pin makros depending on target board, can be extended with other boards (check if enough flash memory is available)
#ifdef SPARKFUN_REDBOARD_TURBO
#define SPI_CS_AD5940_Pin 10
#define AD5940_ResetPin A3
#define AD5940_IntPin 2
#elif defined(ADAFRUIT_FEATHER_M0_WIFI)
#define SPI_CS_AD5940_Pin A5
#define AD5940_ResetPin A4
#define AD5940_IntPin A1
#endif
//*******************************************************
//declarations/definitions
volatile static uint32_t ucInterrupted = 0;       /* Flag to indicate interrupt occurred */
void Ext_Int0_Handler(void);

/**
	@brief Using SPI to transmit N bytes and return the received bytes. This function targets to
         provide a more efficient way to transmit/receive data.
	@param pSendBuffer :{0 - 0xFFFFFFFF}
      - Pointer to the data to be sent.
	@param pRecvBuff :{0 - 0xFFFFFFFF}
      - Pointer to the buffer used to store received data.
	@param length :{0 - 0xFFFFFFFF}
      - Data length in SendBuffer.
	@return None.
**/
void AD5940_ReadWriteNBytes(unsigned char *pSendBuffer, unsigned char *pRecvBuff, unsigned long length)
{
  //set SPI settings for the following transaction
  //speedMaximum: 12MHz found to be max for Adafruit Feather M0, AD5940 rated for max 16MHz clock frequency
  //dataOrder: MSB first
  //dataMode: SCLK idles low/ data clocked on SCLK falling edge --> mode 0
  SPI.beginTransaction(SPISettings(12000000, MSBFIRST, SPI_MODE0));

  for (int i = 0; i < length; i++)
  {
    *pRecvBuff++ = SPI.transfer(*pSendBuffer++);  //do a transfer
  }

  SPI.endTransaction(); //transaction over
}

void AD5940_CsClr(void)
{
  digitalWrite(SPI_CS_AD5940_Pin, LOW);
}

void AD5940_CsSet(void)
{
  digitalWrite(SPI_CS_AD5940_Pin, HIGH);
}

void AD5940_RstSet(void)
{
  digitalWrite(AD5940_ResetPin, HIGH);
}

void AD5940_RstClr(void)
{
  digitalWrite(AD5940_ResetPin, LOW);
}

void AD5940_Delay10us(uint32_t time)
{
  //Warning: micros() only has 4us (for 16MHz boards) or 8us (for 8MHz boards) resolution - use a timer instead?
  unsigned long time_last = micros();
  while (micros() - time_last < time * 10) // subtraction handles the roll over of micros()
  {
    //wait
  }
}

//declare the following function in the ad5940.h file if you use it in other .c files (that include ad5940.h):
//unsigned long AD5940_GetMicros(void);
//used for time tests:
// unsigned long AD5940_GetMicros()
// {
//   return micros();
// }

uint32_t AD5940_GetMCUIntFlag(void)
{
  return ucInterrupted;
}

uint32_t AD5940_ClrMCUIntFlag(void)
{
  ucInterrupted = 0;
  return 1;
}

/* Functions that used to initialize MCU platform */

uint32_t AD5940_MCUResourceInit(void *pCfg)
{
  /* Step1, initialize SPI peripheral and its GPIOs for CS/RST */
  //start the SPI library (setup SCK, MOSI, and MISO pins)
  SPI.begin();
  //initalize SPI chip select pin
  pinMode(SPI_CS_AD5940_Pin, OUTPUT);
  //initalize Reset pin
  pinMode(AD5940_ResetPin, OUTPUT);

  /* Step2: initialize GPIO interrupt that connects to AD5940's interrupt output pin(Gp0, Gp3, Gp4, Gp6 or Gp7 ) */
  //init AD5940 interrupt pin
  pinMode(AD5940_IntPin, INPUT_PULLUP);
  //attach ISR for falling edge
  attachInterrupt(digitalPinToInterrupt(AD5940_IntPin), Ext_Int0_Handler, FALLING);

  //chip select high to de-select AD5940 initially
  AD5940_CsSet();
  AD5940_RstSet();
  return 0;
}

/* MCU related external line interrupt service routine */
//The interrupt handler handles the interrupt to the MCU
//when the AD5940 INTC pin generates an interrupt to alert the MCU that data is ready
void Ext_Int0_Handler()
{
  ucInterrupted = 1;
  /* This example just set the flag and deal with interrupt in AD5940Main function. It's your choice to choose how to process interrupt. */
}
