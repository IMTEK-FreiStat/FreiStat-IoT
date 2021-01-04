/*!
 *****************************************************************************
  @file:    AD590_Arduino_Main_CA.ino
  @author:  D. Bill (adapted from Analog Devices example code)

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

  -----------------------------------------------------------------------------
  @brief:   The Main file for running CA on AD5940
  - content of this file is basically the AD5940Main.c file of the AD5940_Chronomperometric example project
  - currently, only serial output is supported
  - use PLOT_DATA makro (and ensure that DEBUG is NOT defined) for real time plot with corresponfing python application

*****************************************************************************/

#include "ad5940.h"
#include "AD5940.h"

#include "ChronoAmperometric.h"

#include <LibPrintf.h>  //allows to use c-style print() statement instead of Arduino style Serial.print()


//#define DEBUG
#define PLOT_DATA /* use this macro to output data via serial port for python plotting */

#define APPBUFF_SIZE 1000
#ifdef __ICCARM__
#pragma location = "never_retained_ram"
#endif
uint32_t AppBuff[APPBUFF_SIZE];
float LFOSCFreq;
uint32_t IntCount = 0;

/**
 * @brief data ouput function (USB for real time plot with python)
 * @note USB has limited speed, it has impact when sample rate is fast. Try to print some of the data not all of them.
 * @param pData: the buffer stored data (measured cell current) for this application. The data from FIFO has been pre-processed.
 * @param DataCount: The available data count in buffer pData.
 * @return return 0.
*/
int32_t AMPShowResult(float *pData, uint32_t DataCount)
{
  /*static*/ uint32_t index = 0;

  static uint16_t StepIndex = 0; //used to indicate the step to which the data belong

#ifdef PLOT_DATA
    //pack current in a byte array
    float *cur_val = &pData[0];
    uint8_t *byteData = (uint8_t *)cur_val; 
    //write the bytes to serial port
    Serial.write(byteData, 4); //write 4 bytes = current (float)
#endif

#ifdef DEBUG
    //printf("index:%d, Current:%fuA\n", index++, pData[0]);
    printf("%d: %fmV | %fuA\n", StepIndex + 1, PulseVoltage[StepIndex], pData[0]);
#endif

  if (CurrStep != StepIndex) //if CurrStep chaged (after Custom interrupt 1 occured to indicated step change), from now on, the data belongs to this new step
  {
    StepIndex = CurrStep;
  }

  return 0;
}

/* Initialize AD5940 basic blocks like clock */
static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;
  LFOSCMeasure_Type LfoscMeasure;

  /* Use hardware reset */
  AD5940_HWReset();

  /* Platform configuration */
  AD5940_Initialize();
  /* Step1. Configure clock */
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  fifo_cfg.FIFOEn = bFALSE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB; /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = 4;   //will be overwritten in AD5940AMPStructInit()!
  AD5940_FIFOCfg(&fifo_cfg); /* Disable to reset FIFO. */
  fifo_cfg.FIFOEn = bTRUE;
  AD5940_FIFOCfg(&fifo_cfg); /* Enable FIFO here */

  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE); /* Enable all interrupt in Interrupt Controller 1, so we can check INTC flags */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH | AFEINTSRC_ENDSEQ | AFEINTSRC_CUSTOMINT1 | AFEINTSRC_GPT1INT_TRYBRK | AFEINTSRC_DATAFIFOOF, bTRUE); /* Interrupt Controller 0 will control GP0 to generate interrupt to MCU */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Reconfigure GPIO */
  gpio_cfg.FuncSet = GP6_SYNC | GP5_SYNC | GP4_SYNC | GP2_TRIG | GP1_SYNC | GP0_INT;
  gpio_cfg.InputEnSet = AGPIO_Pin2;
  gpio_cfg.OutputEnSet = AGPIO_Pin0 | AGPIO_Pin1 | AGPIO_Pin4 | AGPIO_Pin5 | AGPIO_Pin6;
  gpio_cfg.OutVal = AGPIO_Pin1; //set high to turn off LED
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);

  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK); /* Enable AFE to enter sleep mode. */
  /* Measure LFOSC frequency */
  LfoscMeasure.CalDuration = 1000.0; /* 1000ms used for calibration. */
  LfoscMeasure.CalSeqAddr = 0;
  LfoscMeasure.SystemClkFreq = 16000000.0f; /* 16MHz in this firmware. */
  AD5940_LFOSCMeasure(&LfoscMeasure, &LFOSCFreq);

#ifdef DEBUG
  printf("Measured LFOSC Freq (used for sequencer timing) - rounded: %d Hz\n", (int)(LFOSCFreq + 0.5));
#endif
  return 0;
}

/* !!Change the application parameters here if you want to change it to none-default value */
void AD5940AMPStructInit(void)
{
  AppCHRONOAMPCfg_Type *pAMPCfg;
  AppCHRONOAMPGetCfg(&pAMPCfg);
  /* Configure general parameters */
  pAMPCfg->WuptClkFreq = LFOSCFreq; /* Use measured 32kHz clock freq for accurate wake up timer */
  pAMPCfg->SeqStartAddr = 0;
  pAMPCfg->MaxSeqLen = 512; /* 2kB/4BytesPerCommand = 512 sequencer commands */
  pAMPCfg->RcalVal = 10000.0;

  pAMPCfg->FifoThresh = 10; //determines how many samples get averaged for output!!!
  pAMPCfg->ADCRefVolt = 1.82; /* The real ADC reference voltage. Measure it from capacitor C3 (AD5941 FeatherWing Rev1 Board) with DMM. */

  pAMPCfg->ExtRtia = bFALSE;              /* Set to true if using external Rtia */
  pAMPCfg->ExtRtiaVal = 10000000;         /* Enter external Rtia value here if using one */
  pAMPCfg->LptiaRtiaSel = LPTIARTIA_8K; /* Select TIA gain resistor. */

  pAMPCfg->SensorBias = 0; /* Sensor bias voltage between sense and reference electrodes in mV*/
  pAMPCfg->Vzero = 1300;   /* Set potential at WE, determines the positive and negative range */

  pAMPCfg->LpAmpPwrMod = LPAMPPWR_NORM;  //LPAMPPWR_BOOST3;/* restrict to max. +/- 750 uA cell current*/
  
  /* Configure Pulse*/
  int numCycles = 10; //number of cycles
  pAMPCfg->StepNumber = 3*numCycles;  //each cycle has 3 pulses

  //fill the pulseAmplitude and pulseLength arrays with the voltages for 3 pulses per cycle
  for (int i = 0; i < numCycles; i++)
  {
    pAMPCfg->pulseAmplitude[i*3] = 800; /* Pulse amplitude (WE - RE) (mV) */
    pAMPCfg->pulseAmplitude[i*3+1] = 400;
    pAMPCfg->pulseAmplitude[i*3+2] = -300;
    pAMPCfg->pulseLength[i*3] = 5000; /* Length of voltage pulse in ms */
    pAMPCfg->pulseLength[i*3+1] = 5000;
    pAMPCfg->pulseLength[i*3+2] = 50000;
  }
  
  /* ADC Configuration*/
  pAMPCfg->ADCPgaGain = ADCPGA_1P5;
  pAMPCfg->ADCSinc3Osr = ADCSINC3OSR_2; //Only change according to table 41 in datasheet to use 50Hz filter
  pAMPCfg->ADCSinc2Osr = ADCSINC2OSR_1333;  //Only change according to table 41 in datasheet to use 50Hz filter
}

void AD5940_Main(void)
{
  uint32_t temp;
  AppCHRONOAMPCfg_Type *pAMPCfg;
  AppCHRONOAMPGetCfg(&pAMPCfg);
  AD5940PlatformCfg();

  AD5940AMPStructInit(); /* Configure your parameters in this function */

  AppCHRONOAMPInit(AppBuff, APPBUFF_SIZE); /* Initialize AMP application. Provide a buffer, which is used to store sequencer commands */

//after application init, print the configuration parameters
#ifdef DEBUG
  int Sinc3OSR[] = {5, 4, 2};
  int Sinc2OSR[] = {22, 44, 89, 178, 267, 533, 640, 667, 800, 889, 1067, 1333};
  float PgaGain[] = {1, 1.5, 2, 4, 9};
  printf("---Configuration of CA---\n");
  //conifgured pulse voltages
  printf("Config of Pulses (integer values):\n");
  for (int i = 0; i < pAMPCfg->StepNumber; i++)
  {
    printf("%u: %d mV - %u ms\n", (i + 1), (int)pAMPCfg->pulseAmplitude[i], (uint16_t)pAMPCfg->pulseLength[i]);
  }
  //exact pulse voltages
  printf("DAC outputs (integer values):\n");
  for (int i = 0; i < pAMPCfg->StepNumber; i++)
  {
    printf("%u: %d uV\n", (i + 1), (int)(PulseVoltage[i] * 1000));
  }

  //OSR
  printf("Sinc3OSR: %u\n", Sinc3OSR[pAMPCfg->ADCSinc3Osr]);
  printf("Sinc2OSR: %u\n", Sinc2OSR[pAMPCfg->ADCSinc2Osr]);
  //PGA Gain
  printf("ADC-PGA-Gain = %d.", ((int)PgaGain[pAMPCfg->ADCPgaGain]));
  printf("%d\n", (int)(PgaGain[pAMPCfg->ADCPgaGain] * 10.0 - ((int)PgaGain[pAMPCfg->ADCSinc3Osr] * 10)));
  //Calibrated Rtia
  printf("Calibrated Rtia - Magnitude = %d.", ((int)pAMPCfg->RtiaCalValue.Magnitude));
  int digits = (int)((pAMPCfg->RtiaCalValue.Magnitude * 1000.0 - ((int)pAMPCfg->RtiaCalValue.Magnitude * 1000)) + 0.0005);
  if (digits < 100)
    printf("0"); //one leading 0
  if (digits < 10)
    printf("0"); //two leading 0
  printf("%d Ohm\n", digits);
  //Calibrated Rtia
  //printf("Calibrated Rtia - Magnitude = %d.", ((int)pAMPCfg->RtiaCalValue.Magnitude));
  //printf("%d Ohm\n", (int)((pAMPCfg->RtiaCalValue.Magnitude * 1000.0 - ((int)pAMPCfg->RtiaCalValue.Magnitude * 1000)) + 0.0005));
  //current range
  //check different ADC input voltage ranges for different PGA settings
  if (PgaGain[pAMPCfg->ADCPgaGain] <= 1.5) //PGA = 1, 1.5
    printf("Current range: +/- %d uA\n", (int)(900.0 / PgaGain[pAMPCfg->ADCPgaGain] / pAMPCfg->RtiaCalValue.Magnitude * 1000 + 0.5));
  else if (PgaGain[pAMPCfg->ADCPgaGain] == 2) //PGA = 2
    printf("Current range: +/- %d uA\n", (int)(600.0 / PgaGain[pAMPCfg->ADCPgaGain] / pAMPCfg->RtiaCalValue.Magnitude * 1000 + 0.5));
  else if (PgaGain[pAMPCfg->ADCPgaGain] == 4) //PGA = 4
    printf("Current range: +/- %d uA\n", (int)(300.0 / PgaGain[pAMPCfg->ADCPgaGain] / pAMPCfg->RtiaCalValue.Magnitude * 1000 + 0.5));
  else if (PgaGain[pAMPCfg->ADCPgaGain] == 9) //PGA = 9
    printf("Current range: +/- %d uA\n", (int)(133.0 / PgaGain[pAMPCfg->ADCPgaGain] / pAMPCfg->RtiaCalValue.Magnitude * 1000 + 0.5));
  //resolution
  float resolution = pAMPCfg->ADCRefVolt / (32768 * pAMPCfg->RtiaCalValue.Magnitude * PgaGain[pAMPCfg->ADCPgaGain]) * 1E9; //resolution in nA
  if (resolution < 1)
  {
    //output resolution in pA
    resolution *= 1E3; //convert to pA
    printf("Current resolution: %d pA\n", (int)resolution);
  }
  //output resolution in nA
  else
  {
    printf("Current resolution: %d nA\n", (int)resolution);
  }
#endif

//send CA config data to python application
#ifdef PLOT_DATA
  AppCHRONOAMPGetCfg(&pAMPCfg);
#ifndef DEBUG
  int Sinc3OSR[] = {5, 4, 2};
  int Sinc2OSR[] = {22, 44, 89, 178, 267, 533, 640, 667, 800, 889, 1067, 1333};
#endif
  uint8_t byteData[30]; //buffer for sending bytes to python application

  //send exact pulse voltages (in mV)
  uint8_t *pulseBytes = (uint8_t *)PulseVoltage; //cast pointer to the exact pulse voltage array to a byte pointer
  //write  all the bytes to serial port
  Serial.write(pulseBytes, (MAX_STEPS * 4)); //each entry in PulseVoltage has 4 bytes (float), MAX_STEPS entries

  //calculate duration of CA (needed to set x axis limits)
  uint16_t CA_duration = 0;
  for (int i = 0; i < pAMPCfg->StepNumber; i++)
  {
    CA_duration += (int) pAMPCfg->pulseLength[i]/1000+0.5;
  }

  //pack  important config data into buffer
  byteData[0] = ((int16_t)pAMPCfg->StepNumber & 0xFF00) >> 8;
  byteData[1] = (int16_t)pAMPCfg->StepNumber & 0x00FF;
  byteData[2] = ((int16_t)Sinc3OSR[pAMPCfg->ADCSinc3Osr] & 0xFF00) >> 8;
  byteData[3] = (int16_t)Sinc3OSR[pAMPCfg->ADCSinc3Osr] & 0x00FF;
  byteData[4] = ((int16_t)Sinc2OSR[pAMPCfg->ADCSinc2Osr] & 0xFF00) >> 8;
  byteData[5] = (int16_t)Sinc2OSR[pAMPCfg->ADCSinc2Osr] & 0x00FF;
  byteData[6] = (CA_duration & 0xFF00) >> 8;
  byteData[7] = CA_duration & 0x00FF;

  //write the bytes to serial port
  Serial.write(&byteData[0], 8);

  //delay to allow python to set everything up
  delay(3000);
#endif

#ifdef DEBUG
  printf("---start of CA---\n");
#endif

  AppCHRONOAMPCtrl(CHRONOAMPCTRL_PULSETEST, 0); /* Control AMP measurement. AMPCTRL_PULSETEST carries out pulse test*/

  while (1)
  {
    /* Check if interrupt flag which will be set when interrupt occurred. */
    if (AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag(); /* Clear this flag */
      temp = APPBUFF_SIZE;
      AppCHRONOAMPISR(AppBuff, &temp); /* Deal with it and provide a buffer to store data we got */

      AMPShowResult((float *)AppBuff, temp);

      if (pAMPCfg->EndSeq)
      {

#ifdef DEBUG
        printf( "---end of CA---\n");
#endif
        while (1)
          ;
      }
    }
  }
}

//***************************SETUP***************************
void setup()
{
  //init baud rate of UART, irrelevant for SAMD21 because it uses USB CDC serial port (built in USB controller)
  Serial.begin(115200);

  //wait until COM port is opened (e.g. by python)
  while (!Serial)
    ;

  pinMode(A2, INPUT_PULLUP); //allow AD5940 to set the LED on this pin

  //init GPIOs (SPI, AD5940 Reset and Interrupt)
  AD5940_MCUResourceInit(0);

  AD5940_Main();
}

//***************************LOOP***************************
void loop()
{
  //will never be reached!
}
