/*!
 *****************************************************************************
  @file:    AD590_Arduino_Main_CV.ino
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
  @brief:   The Main file for running CV/LSV on AD5940
  - performs cyclic or linear sweep voltammetry and sends data (voltage, current) to Computer (USB) for real-time plot with corresponding python application
  - content of this file is basically the AD5940Main.c file of the AD5940_Ramp example project
  - content from main.c (init of serial communication and mcu resource) placed in setup()
  - setup() inits the AD5941 AFE and sets the ramp parameters to values configurable with AD5940RampStructInit()
  - loop() does not do anything
  - use the makro definitions to determine operation variants
  - data output options: USB
  - Cyclic Voltammetry (CV): AppRampCfg.bRampOneDir = bFALSE
  - Linear Sweep Voltammetry (LSV): AppRampCfg.bRampOneDir = bTRUE
  - Potentiometry() method in RampTest.c file can be used to measure Open Circuit Potential (OCP)
  - CV/LSV only starts after opening a serial port on computer to communicate with MCU
  -----------------------------------------------------------------------------
  @todo:
  - low power DAC offset calibration (remove sensor bias from calibration routine)
  - ADC PGA gain calibration does not work properly
  - make DAC code re-writes during runtime more time efficient (prevent floating point arithmetic in AppRAMPSeqDACCtrlGen() and RampDacRegUpdate())
*****************************************************************************/
#include "ad5940.h"
#include "RampTest.h"
#include <LibPrintf.h> //allows to use c-style print() statement instead of Arduino style Serial.print()

#define PLOT_DATA /* use this macro to output data via serial port for python plotting */
//#define DEBUG /* use this macro to output debug info*/

/**
   User could configure following parameters
**/
#define APPBUFF_SIZE 1024

uint32_t AppBuff[APPBUFF_SIZE]; //buffer to fetch AD5940 samples
float LFOSCFreq;                /* Measured LFOSC frequency */

/**
 * @brief data ouput function
 * @param pData: the buffer stored data (voltage of a step and corresponding cell current) for this application. The data from FIFO has been pre-processed.
 * @param DataCount: The available data count in buffer pData.
 * @return return 0.
*/
static int32_t RampShowResult(float *pData, uint32_t DataCount)
{
  /* Print data*/
  for (int i = 0; i < DataCount * 2; i += 2)
  {
    //pack voltage and current in a byte array
    uint8_t *byteData = (uint8_t *)pData; //cast pointer to the pData array to a byte pointer
    //write the bytes to serial port
    Serial.write(byteData, 8); //first 4 bytes = voltage, other 4 bytes = current
  }
  return 0;
}

/**
 * @brief The general configuration to AD5940 like FIFO/Sequencer/Clock. 
 * @note This function will firstly reset AD5940 using reset pin.
 * @return return 0.
*/
static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  SEQCfg_Type seq_cfg;
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;
  LFOSCMeasure_Type LfoscMeasure;

  /* Use hardware reset */
  AD5940_HWReset();
  AD5940_Initialize(); /* Call this right after AFE reset */
  /* Platform configuration */
  /* Step1. Configure clock */
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  /* Configure FIFO and Sequencer */
  fifo_cfg.FIFOEn = bTRUE; /* We will enable FIFO after all parameters configured */
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_2KB; /* 2kB for FIFO, The reset 4kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_SINC3; /* */
  fifo_cfg.FIFOThresh = 4;          /*  Don't care, set it by application paramter */
  AD5940_FIFOCfg(&fifo_cfg);
  seq_cfg.SeqMemSize = SEQMEMSIZE_4KB; /* 4kB SRAM is used for sequencer, others for data FIFO */
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bTRUE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);
  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE); /* Enable all interrupt in INTC1, so we can check INTC flags */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH | AFEINTSRC_ENDSEQ | AFEINTSRC_CUSTOMINT0 | AFEINTSRC_CUSTOMINT1 | AFEINTSRC_GPT1INT_TRYBRK | AFEINTSRC_DATAFIFOOF, bTRUE);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Configure GPIO */
  gpio_cfg.FuncSet = GP0_INT | GP1_GPIO | GP2_SYNC; /* GPIO1 indicates AFE is in sleep state. GPIO2 indicates ADC is sampling. */
  gpio_cfg.InputEnSet = 0;
  gpio_cfg.OutputEnSet = AGPIO_Pin0 | AGPIO_Pin1 | AGPIO_Pin2;
  gpio_cfg.OutVal = AGPIO_Pin1; //set high to turn off LED
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);
  /* Measure LFOSC frequency */
  /**@note Calibrate LFOSC using system clock. The system clock accuracy decides measurement accuracy. Use XTAL to get better result. */
  LfoscMeasure.CalDuration = 1000.0;        /* 1000ms used for calibration. */
  LfoscMeasure.CalSeqAddr = 0;              /* Put sequence commands from start address of SRAM */
  LfoscMeasure.SystemClkFreq = 16000000.0f; /* 16MHz in this firmware. */
  AD5940_LFOSCMeasure(&LfoscMeasure, &LFOSCFreq);
#ifdef DEBUG
  printf("Measured LFOSC Freq (used for sequencer timing) - rounded: %d Hz\n", (int)(LFOSCFreq + 0.5));
#endif
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK); /*  */
  return 0;
}

/**
 * @brief The interface for user to change application parameters. All parameters belong to the AppRAMPCfg (see RampTest.c file)
 * @return return 0.
*/
void AD5940RampStructInit(void)
{
  AppRAMPCfg_Type *pRampCfg;

  AppRAMPGetCfg(&pRampCfg);
  /* Step1: configure general parameters */
  pRampCfg->SeqStartAddr = 0x10;      /* leave 16 commands for LFOSC calibration.  */
  pRampCfg->MaxSeqLen = 1024 - 0x10;  /* 4kB/4 = 1024  */
  pRampCfg->RcalVal = 10000.0;        /* 10kOhm RCAL */
  pRampCfg->ADCRefVolt = 1820.0f;     /* The real ADC reference voltage. Measure it from capacitor C3 (AD5941 FeatherWing Rev1 Board) with DMM. */
  pRampCfg->FifoThresh = 100;         /* Maximum value is 2kB/4-1 = 512-1. Set it to higher value to save power. */
  pRampCfg->SysClkFreq = 16000000.0f; /* System clock is 16MHz by default */
  pRampCfg->LFOSCClkFreq = LFOSCFreq; /* LFOSC frequency */
  /* Configure ramp signal parameters */
  pRampCfg->RampStartVolt = 500.0f;  /* in mV*/
  pRampCfg->RampPeakVolt1 = 1050.0f; /* If FIX_WE_POT defined, make sure (|RampPeakVolt1 - RampPeakVolt2| + 35mV) <= (VzeroHighLevel-VzeroLowLevel) */
  pRampCfg->RampPeakVolt2 = -610.0f; //-100.0f;
  pRampCfg->VzeroLimitHigh = 2400;   /* 2.2V */
  pRampCfg->VzeroLimitLow = 200;     /* 0.4V */
  pRampCfg->Estep = 2;
  pRampCfg->ScanRate = 50;
  pRampCfg->CycleNumber = 10;

  pRampCfg->ADCSinc3Osr = ADCSINC3OSR_4;
  pRampCfg->ADCSinc2Osr = ADCSINC2OSR_667;

  pRampCfg->SampleDelay = 7.0f;           /* 7ms. Time between update DAC and ADC sample. Unit is ms. SampleDelay > 1.0ms is acceptable.*/
  pRampCfg->LpAmpPwrMod = LPAMPPWR_NORM;  /* restrict to max. +/- 750 uA cell current*/
  pRampCfg->LPTIARtiaSel = LPTIARTIA_32K; /* Maximum current decides RTIA value: Imax = 0.9V / RTIA */
  pRampCfg->LPTIARloadSel = LPTIARLOAD_SHORT;
  pRampCfg->AdcPgaGain = ADCPGA_1P5;

  pRampCfg->bRampOneDir = bFALSE; //activate LSV instead of CV

#ifdef DEBUG
  printf("Ramp Parameters:\n");
  if (pRampCfg->bRampOneDir)
  {
    printf("Setup for Linear Sweep Voltammetry!\n");
  }
  else
  {
    printf("Setup for Cyclic Voltammetry!\n");
  }
  printf("- Vstart = %d mV\n", (int)pRampCfg->RampStartVolt);
  printf("- V1 = %d mV\n", (int)pRampCfg->RampPeakVolt1);
  printf("- V2 = %d mV\n", (int)pRampCfg->RampPeakVolt2);
  printf("- Estep = %d mV\n", (int)pRampCfg->Estep);
  printf("- Scan Rate = %d mV/s\n", (int)pRampCfg->ScanRate);
  printf("- Number of Cycles = %d\n", (int)pRampCfg->CycleNumber);
  int Sinc3OSR[] = {2, 4, 5};
  int Sinc2OSR[] = {22, 44, 89, 178, 267, 533, 640, 667, 800, 889, 1067, 1333};
  int SampleRate = (int)(800000.0 / Sinc3OSR[pRampCfg->ADCSinc3Osr] / Sinc2OSR[pRampCfg->ADCSinc2Osr] + 0.5);
  printf("- Sampling Freq (rounded) = %d Sps\n", (int)SampleRate);
  //check timing parameters
  //necessary time after each step: DAC code write (~3.1ms for StepsPerBlock = 1) + get remaining FIFO data from last step and process (~3.2ms / 50 samples in FIFO) + print output (~0.5ms)
  //available time after each step: SampleDelay + min(Estep/ScanRate in ms - SampleDelay, FIFOThresh / fSample)
  float a = pRampCfg->Estep / pRampCfg->ScanRate * 1000.0 - pRampCfg->SampleDelay;
  float b = pRampCfg->FifoThresh / ((float)SampleRate) * 1000;
  //float c = min(a,b);
  if ((3.1 + 3.2 * pRampCfg->FifoThresh / 50 + 0.5) >=
      (pRampCfg->SampleDelay + min(a, b)))
  {
    printf("WARNING: Timing critical - Increase Estep, decrease ScanRate or adapt FIFO threshold\n");
  }
#endif
}

void AD5940_Main(void)
{
  uint32_t temp;
  AppRAMPCfg_Type *pRampCfg;

  AppRAMPInit(AppBuff, APPBUFF_SIZE); /* Initialize RAMP application. Provide a buffer, which is used to store sequencer commands */

  //send ramp data to python application
#ifdef PLOT_DATA
  AppRAMPGetCfg(&pRampCfg);
  int Sinc3OSR[] = {2, 4, 5};
  int Sinc2OSR[] = {22, 44, 89, 178, 267, 533, 640, 667, 800, 889, 1067, 1333};
  //pack voltage and current in a byte array
  uint8_t byteData[30];
  byteData[0] = ((int16_t)pRampCfg->RampStartVolt & 0xFF00) >> 8;
  byteData[1] = (int16_t)pRampCfg->RampStartVolt & 0x00FF;
  byteData[2] = ((int16_t)pRampCfg->RampPeakVolt1 & 0xFF00) >> 8;
  byteData[3] = (int16_t)pRampCfg->RampPeakVolt1 & 0x00FF;
  byteData[4] = ((int16_t)pRampCfg->RampPeakVolt2 & 0xFF00) >> 8;
  byteData[5] = (int16_t)pRampCfg->RampPeakVolt2 & 0x00FF;
  byteData[6] = ((int16_t)pRampCfg->Estep & 0xFF00) >> 8;
  byteData[7] = (int16_t)pRampCfg->Estep & 0x00FF;
  byteData[8] = ((int16_t)pRampCfg->ScanRate & 0xFF00) >> 8;
  byteData[9] = (int16_t)pRampCfg->ScanRate & 0x00FF;
  byteData[10] = ((int16_t)pRampCfg->CycleNumber & 0xFF00) >> 8;
  byteData[11] = (int16_t)pRampCfg->CycleNumber & 0x00FF;
  byteData[12] = ((int16_t)Sinc3OSR[pRampCfg->ADCSinc3Osr] & 0xFF00) >> 8;
  byteData[13] = (int16_t)Sinc3OSR[pRampCfg->ADCSinc3Osr] & 0x00FF;
  byteData[14] = ((int16_t)Sinc2OSR[pRampCfg->ADCSinc2Osr] & 0xFF00) >> 8;
  byteData[15] = (int16_t)Sinc2OSR[pRampCfg->ADCSinc2Osr] & 0x00FF;
  byteData[16] = ((int16_t)pRampCfg->StepNumber & 0xFF00) >> 8;
  byteData[17] = (int16_t)pRampCfg->StepNumber & 0x00FF;
  //write the bytes to serial port
  Serial.write(&byteData[0], 18);
  //delay to allow python to set everything up
  delay(3000);
#endif

#ifdef DEBUG
  printf("All initialized.\n");
  printf("---start of voltammetry ---\n");
#endif
  AppRAMPCtrl(APPCTRL_START, 0); /* Control RAMP measurement to start. Second parameter has no meaning with this command. */

  while (!pRampCfg->bTestFinished)
  {
    AppRAMPGetCfg(&pRampCfg);
    if (AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag();
      temp = APPBUFF_SIZE;
      AppRAMPISR(AppBuff, &temp); //temp now holds the number of calculated means (>0, if at least one step was finished within the current data set from FIFO)
      //print data in case at least one step was finished
      if (temp > 0)
      {
        RampShowResult((float *)AppBuff, temp);
      }
    }
  }

  //end of test
#ifdef DEBUG
  printf("---end of ramp test---\n");
#endif

  //after test finished, reset flag to be able to start new test
  pRampCfg->bTestFinished = bFALSE;
}

//***************************SETUP***************************
void setup()
{
  Serial.begin(115200);

  //wait until COM port is opened (e.g. by python)
  while (!Serial)
    ;

  pinMode(A2, INPUT_PULLUP); //allow AD5940 to set the LED on this pin

  //init GPIOs (SPI, AD5940 Reset and Interrupt)
  AD5940_MCUResourceInit(0);

  //configure AFE
  AD5940PlatformCfg();
  //init application with pre-defined parameters
  AD5940RampStructInit();

  //run voltammetry once
  AD5940_Main();
}

//***************************LOOP***************************
void loop()
{
  //do nothing forever
}