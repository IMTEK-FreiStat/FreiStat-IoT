/*!
 *****************************************************************************
 @file:    RampTest.h
 @author:  D. Bill (adapted from Analog Devices example code)
 @brief:   Ramp Test header file.
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
#ifndef _RAMPTEST_H_
#define _RAMPTEST_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "ad5940.h"
#include <stdio.h>
#include "string.h"
#include "math.h"

/* Do not modify following parameters */
#define ALIGIN_VOLT2LSB 1                         /* Set it to 1 to align each voltage step to 1LSB of DAC. 0: step code is fractional. */
#define DAC12BITVOLT_1LSB (2200.0f / 4095)        //mV
#define DAC6BITVOLT_1LSB (DAC12BITVOLT_1LSB * 64) //mV

//#define START_EQUI_POT  /* Define it to change the Start voltage of the ramp to the measured equilibrium potential (WE-RE) */
#define FIX_WE_POT /* Define it to fix the potential at the WE (Vzero) instead of switching between VzeroHighLevel and VzeroLowLevel -> prevents offset artefact in cell current when cell voltage crosses zero due to DAC inaccuracy but limits cell voltage range!*/
  /**
 * The Ramp application related paramter structure
*/
  typedef struct
  {
    /* Common configurations for all kinds of Application. */
    BoolFlag bParaChanged;    /**< Indicate to generate sequence again. It's auto cleared by AppBIAInit */
    uint32_t SeqStartAddr;    /**< Initialaztion sequence start address in SRAM of AD5940  */
    uint32_t MaxSeqLen;       /**< Limit the maximum sequence.   */
    uint32_t SeqStartAddrCal; /**< Not used for Ramp.Calibration sequence start address in SRAM of AD5940 */
    uint32_t MaxSeqLenCal;    /**< Not used for Ramp. */
                              /* Application related parameters */
    float LFOSCClkFreq;       /**< The clock frequency of Wakeup Timer in Hz. Typically it's 32kHz. Leave it here in case we calibrate clock in software method */
    float SysClkFreq;         /**< The real frequency of system clock */
    float AdcClkFreq;         /**< The real frequency of ADC clock */
    float RcalVal;            /**< Rcal value in Ohm */
    float ADCRefVolt;         /**< The real ADC voltage in mV. */
    BoolFlag bTestFinished;   /**< Variable to indicate ramt test has finished >*/
    /* Describe Ramp signal */
    float RampStartVolt;  /**< The start voltage (WE-RE) of ramp signal in mV */
    float RampPeakVolt1;  /**< The first peak (maximum or minimum) voltage of ramp in mV */
    float RampPeakVolt2;  /**< The second peak (minimum or maximum) voltage of ramp in mV */
    float Estep;          /**< The potential difference between each step in mV --> determines StepNumber. Ideally a multiple of DAC12BITVOLT_1LSB*/
    float ScanRate;       /**< Slope of the ramp in mV/sec --> determines RampDuration*/
    float VzeroLimitHigh; /**< The limit for the high level voltage of Vzero in mV. Set it to 2400mV by default. range = (VzeroLimitHigh - VzeroLimitLow) */
    float VzeroLimitLow;  /**< The limit for the low level voltage of Vzero in mV. Set it to 200mV by default. range = (VzeroLimitHigh - VzeroLimitLow)  */
    uint16_t CycleNumber; /**< The number of cycles to repeat the ramp test */
    /* Ramp parameters that are being calculated during runtime */
    uint32_t StepNumber;   /**< Total number of steps per cycle. Set during runtime!*/
    uint32_t RampDuration; /**< Ramp signal duration(total time) in ms. Set during runtime! */
    uint32_t StepsToPeak1; /**< Number of steps between RampStartVolt and RampPeakVolt1. Set during runtime!*/
    uint32_t StepsToPeak2; /**< Number of steps between RampStartVolt and RampPeakVolt2. Set during runtime!*/
    float VzeroHighLevel;  /**< The high level voltage of Vzero in mV. */
    float VzeroLowLevel;   /**< The low level voltage of Vzero in mV. */
    /* Receive path configuration */
    float SampleDelay;       /**< The time delay between update DAC and start ADC */
    uint32_t LpAmpPwrMod;    /**< Defines the power mode of PA and LPTIA -> affects max cell current! */
    uint32_t LPTIARtiaSel;   /**< Select RTIA. Maximum current decides RTIA value */
    uint32_t LPTIARloadSel;  /**< Select Rload */
    float ExternalRtiaValue; /**< The optional external RTIA value in Ohm. Disconnect internal RTIA to use external RTIA. When using internal RTIA, this value is ignored. */
    uint32_t AdcPgaGain;     /**< PGA Gain select from GNPGA_1, GNPGA_1_5, GNPGA_2, GNPGA_4, GNPGA_9 !!! We must ensure signal is in range of +-1.5V which is limited by ADC input stage */
    uint8_t ADCSinc3Osr;     /**< Sinc3 output is fed into Sinc2 filter. */
    uint8_t ADCSinc2Osr;     /**< We use data from Sinc2 filter. NOTE: Make sure that data output rate (for FIFO) is way less than it takes to read (~6usec), process and output the data at MCU*/
    /* Digital related */
    uint32_t FifoThresh;    /**< FIFO Threshold value. Must be set to 1 in order to distinguish steps! [Maximum value is 2kB/4-1 = 512-1. Set it to higher value to save power.]*/
                            /* Private variables for internal usage */
    BoolFlag RAMPInited;    /**< If the program run firstly, generated initialization sequence commands */
    fImpPol_Type RtiaValue; /**< Calibrated Rtia value */
    SEQInfo_Type InitSeqInfo;
    SEQInfo_Type ADCSeqInfo;
    BoolFlag bFirstDACSeq;   /**< Init DAC sequence */
    SEQInfo_Type DACSeqInfo; /**< The first DAC update sequence info */
    uint32_t CurrStepPos;    /**< Current position */
    float DACCodePerStep;    /**<  */
    float CurrRampCode;      /**<  Current cell voltage (WE-RE) in mV divided by DAC12BITVOLT_1LSB (difference between WE and RE as 12bit DAC code)*/
    uint32_t CurrVzeroCode;
    BoolFlag bDACCodeInc;  /**< Increase DAC code.  */
    BoolFlag StopRequired; /**< After FIFO is ready, stop the measurement sequence */
    enum _RampState
    {
      RAMP_STATE0 = 0,
      RAMP_STATE1,
      RAMP_STATE2,
      RAMP_STATE3,
      RAMP_STATE4,
      RAMP_STOP
    } RampState;
    BoolFlag bRampOneDir; /**< Ramp in a single direction, ramp sweeps from RampStartVolt to RampPeakVolt1, no return to RampStartVolt, RampPeakVolt2 has no effect*/
  } AppRAMPCfg_Type;

#define APPCTRL_START 0
#define APPCTRL_STOPNOW 1
#define APPCTRL_STOPSYNC 2
#define APPCTRL_SHUTDOWN 3 /**< Note: shutdown here means turn off everything and put AFE to hibernate mode. The word 'SHUT DOWN' is only used here. */

  AD5940Err AppRAMPInit(uint32_t *pBuffer, uint32_t BufferSize);
  AD5940Err AppRAMPGetCfg(void *pCfg);
  AD5940Err AppRAMPISR(void *pBuff, uint32_t *pCount);
  AD5940Err AppRAMPCtrl(uint32_t Command, void *pPara);

#ifdef __cplusplus
}
#endif
#endif
