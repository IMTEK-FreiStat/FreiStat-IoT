/*!
*****************************************************************************
@file:    ChronoAmperometric.h
@author:  Author: D. Bill (adapted from Analog Devices example code)
@brief:   ChronoAmperometric measurement header file.
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
#ifndef _CHRONOAMPEROMETRIC_H_
#define _CHRONOAMPEROMETRIC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "ad5940.h"
#include "stdio.h"
#include "string.h"
#include "math.h"

#define DAC12BITVOLT_1LSB   (2200.0f/4095)  //mV
#define DAC6BITVOLT_1LSB    (DAC12BITVOLT_1LSB*64)  //mV
#define MAX_STEPS 200  //maximally 20 steps can be used for CA

extern volatile uint16_t CurrStep;
extern float PulseVoltage[MAX_STEPS];

typedef struct
{
/* Common configurations for all kinds of Application. */
  BoolFlag bParaChanged;        /* Indicate to generate sequence again. It's auto cleared by AppCHRONOAMPInit */
  uint32_t SeqStartAddr;        /* Initialaztion sequence start address in SRAM of AD5940  */
  uint32_t MaxSeqLen;           /* Limit the maximum sequence.   */
  uint32_t SeqStartAddrCal;     /* Measurement sequence start address in SRAM of AD5940 */
  uint32_t MaxSeqLenCal;
  
/* Application related parameters */ 
  BoolFlag ReDoRtiaCal;         /* Set this flag to bTRUE when there is need to do calibration. */
  float SysClkFreq;             /* The real frequency of system clock */
  float WuptClkFreq;            /* The clock frequency of Wakeup Timer in Hz. Typically it's 32kHz. Leave it here in case we calibrate clock in software method */
  float AdcClkFreq;             /* The real frequency of ADC clock */
  uint32_t FifoThresh;           /* FIFO threshold.*/   

  float RcalVal;                /* Rcal value in Ohm */
  uint32_t PwrMod;              /* Control Chip power mode(LP/HP) */
  uint16_t StepNumber;          /* Number of voltage steps (pulses) in the CA, must not be greater than MAX_STEPS. Only the first StepNumber values in pulseAmplitude[] will be executed.*/
  uint32_t pulseLength[MAX_STEPS];          /* length of the pulses in ms, Max value: 67100 (67.1s)*/

/* Receive path configuration */
  uint32_t LpAmpPwrMod;    /**< Defines the power mode of PA and LPTIA -> affects max cell current! */ 
  uint32_t ADCPgaGain;          /* PGA Gain select from GNPGA_1, GNPGA_1_5, GNPGA_2, GNPGA_4, GNPGA_9 !!! We must ensure signal is in range of +-1.5V which is limited by ADC input stage */   
  uint8_t ADCSinc3Osr;          /* SINC3 OSR selection. ADCSINC3OSR_2, ADCSINC3OSR_4 */
  uint8_t ADCSinc2Osr;          /* SINC2 OSR selection. ADCSINC2OSR_22...ADCSINC2OSR_1333 */
  uint32_t DataFifoSrc;         /* DataFIFO source. DATATYPE_ADCRAW, DATATYPE_SINC3 or DATATYPE_SINC2*/
  uint32_t LptiaRtiaSel;        /* Use internal RTIA, select from RTIA_INT_200, RTIA_INT_1K, RTIA_INT_5K, RTIA_INT_10K, RTIA_INT_20K, RTIA_INT_40K, RTIA_INT_80K, RTIA_INT_160K */
  uint32_t LpTiaRf;             /* Rfilter select */
  uint32_t LpTiaRl;             /* SE0 Rload select */
  fImpPol_Type RtiaCalValue;           /* Calibrated Rtia value */
  BoolFlag ExtRtia;             /* Use internal or external Rtia */
  uint32_t HstiaRtiaSel;        /* Use internal RTIA, select from RTIA_INT_200, RTIA_INT_1K, RTIA_INT_5K, RTIA_INT_10K, RTIA_INT_20K, RTIA_INT_40K, RTIA_INT_80K, RTIA_INT_160K */
  uint32_t CtiaSel;             /* Select CTIA in pF unit from 0 to 31pF */
  
/* LPDAC Config */
  float Vzero;                  /* Voltage on SE0 pin and Vzero. Negative cell voltage range = 2400mV - Vzero, Positive cell voltage range = Vzero - 200mV  */
  float Vbias;                  /* Voltage on CE0 and PA. Set during CA */
	float SensorBias;             /* Used as offset during CA. Sensor bias voltage = VSE0 - VRE0 */
  float pulseAmplitude[MAX_STEPS];                  /* voltages of the pulses in mV*/
  float ADCRefVolt;               /*Vref value */
  float ExtRtiaVal;							/* External Rtia value if using one */
	
  BoolFlag EndSeq;
	
  BoolFlag bMeasureTransient;
  BoolFlag CHRONOAMPInited;                       /* If the program run firstly, generated sequence commands */
  SEQInfo_Type InitSeqInfo;
  SEQInfo_Type TransientSeqInfo;
  BoolFlag StopRequired;          /* After FIFO is ready, stop the measurement sequence */
  uint32_t FifoDataCount;         /* Count how many times impedance have been measured */
/* End */
}AppCHRONOAMPCfg_Type;

/**
 * int32_t type Impedance result in Cartesian coordinate 
*/
typedef struct
{
  float Current;
  float Voltage;
}fAmpRes_Type;



#define CHRONOAMPCTRL_STOPNOW        1
#define CHRONOAMPCTRL_STOPSYNC       2
#define CHRONOAMPCTRL_SHUTDOWN       4   /* Note: shutdown here means turn off everything and put AFE to hibernate mode. The word 'SHUT DOWN' is only used here. */
#define CHRONOAMPCTRL_PULSETEST      5
AD5940Err AppCHRONOAMPGetCfg(void *pCfg);
AD5940Err AppCHRONOAMPInit(uint32_t *pBuffer, uint32_t BufferSize);
AD5940Err AppCHRONOAMPISR(void *pBuff, uint32_t *pCount);
AD5940Err AppCHRONOAMPCtrl(int32_t AmpCtrl, void *pPara);
uint32_t AppCHRONOAMPCalcDataNum(uint32_t time);
float AppCHRONOAMPCalcVoltage(uint32_t ADCcode);
float AppCHRONOAMPCalcCurrent(uint32_t ADCcode);

#ifdef __cplusplus
}
#endif
#endif
