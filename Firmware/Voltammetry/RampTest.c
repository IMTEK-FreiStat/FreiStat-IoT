/*!
 *****************************************************************************
 @file:    RampTest.c
 @author:  D. Bill (adapted from Analog Devices example code)
 @brief:   Functions for controlling voltammetric measurements and process data
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

/** @addtogroup AD5940_System_Examples
 * @{
 *    @defgroup Ramp_Test_Example
 *    @brief Using sequencer to generate ramp signal and control ADC to sample data.
 *    @details
 * @note Need to update code when runs at S2 silicon.
 * @todo update LPDAC switch settings for S2 and LPDAC 1LSB bug.
 * @todo Calibrate ADC/PGA firstly to get accurate current. (Voltage/Rtia = Current)
 * @note The method to calculate LPDAC ouput voltage
 *        - #define LSB_DAC12BIT (2.2V/4095)
 *        - #define LSB_DAC6BIT  (2.2V/4095*64)
 *        - Volt_12bit = Code12Bit*LSB_DAC12BIT + 0.2V
 *        - Volt_6bit = Code6Bit*LSB_DAC6BIT + 0.2V
 *
 * # Ramp Signal Parameters definition
 *
 * @code
 * Cell voltage (WE - RE): 
 *     RampPeakVolt1  -->            /\
 *                                  /  \
 *                                 /    \
 *                                /      \
 *  RampStartVolt   -->         _/        \          _
 *                                         \        /
 *                                          \      /
 *                                           \    /
 *                                            \  /
 *    RampPeakVolt2   -->                      \/
 *
 * Vzero (WE): 
 * This potential is set with 6bit DAC depending on the sign of the cell voltage
 * If there is no limitation on Vzero, Set VzeroLimitHigh to 2.4 and VzeroLimitLow to 0.2V
 * It can be choosen with the FIX_WE_POT makro (RampTest.h) wether the potential at the working electrode is allowed to change (artefact when cell potential crosses 0V) or not
 * If it is allowed to change (possible range = +/- 2200 mV) it results in the following curve:
 * 1) positive cell voltage: WE is at VzeroHighLevel so that WE > RE
 * 2) negative cell voltage: WE is at VzeroLowLevel so that WE < RE
 * Voltage VzeroHighLevel-> _______          ____
 *                                 |        |
 * Voltage VzeroLowLevel  -->      |________|
 *
 * If it is NOT allowed to change (possible range = +/- 1100 mV between +/- 2200 mV absolute boarders), a constant voltage level is calculated depending on the desired peak voltages in the sweep
 * 
 * Vbias (RE):
 * This potential is set with 12bit DAC so that (WE-RE) corresponds to the needed cell voltage
 *                                /|        |\
 *                               / |        | \         
 *                         _    /  |        |  \_               
 *                          \  /   |        |   
 *                           \/    |        |
 *                                 |   /\   |
 *                                 |  /  \  |
 *                                 | /    \ |
 *                                 |/      \|
 * 
 *
 * RampState define:      S0 | S1  | S2 |S3 |  S4 |
 * S0: sweep from Start to Peak1
 * S1: sweep from Peak1 back to Start
 * S2: sweep from Start to Peak2
 * S3: sweep from Peak2 back to Start
 * @endcode
 *
 *  # The sequencer method to do Ramp test.
 * The Ramp test need to update DAC data in real time to generate required waveform, and control ADC to start sample data. \n
 * We used two kinds of sequence to realize it. One is to control DAC where SEQ0 and SEQ1 are used, another sequence SEQ2 controls ADC.
 *  ## Sequence Allocation
 * SEQ3 is used to initialize AD5940.\n
 * SEQ0/1 is used to stop ADC, output interrupt to indicate new step to MCU (so that it can process values from finished step) and generate next voltage step.\n
 * SEQ2 is used to startup ADC, then during the whole step, the current gets sampled until the next DAC update command (SEQ0/1) stops the ADC
 *
 * |SRAM allocation|||
 * |------------|----------------|---------|
 * |SequenceID  | AddressRange   |  Usage  |
 * |SEQID_3     | 0x0000-0xzzzz  | Initialization sequence|
 * |SEQID_2     | 0xzzzz-0xyyyy  | ADC control sequence, run this sequence will start the ADC|
 * |SEQID_0/1   | 0xyyyy-end     | DAC update sequence. If size if not enough for all steps, use it like a Ping Pong buffer.|
 * Where 0xzzzz equals to SEQ3 length, 0xyyyy equals to sum of SEQ2 and SEQ3 length.
 * In one word, put SEQ2 commands right after SEQ3. Don't waste any SRAM resource.
 *  ##Sequencer Running Order
 * The sequencer running order is set to firstly update DAC then start ADC. Repeat this process until all waveform generated.
 * Below is explanation of sequencer running order.
 * @code
 * DAC voltage changes with sequencer, assume each step is 0.05V start from 0.2V
 * 400mV->                               _______
 * 350mV->                       _______/       \_______
 * 300mV->               _______/                       \_________
 * 250mV->       _______/
 * 200mV->    __/
 * Update DAC:  ↑       ↑       ↑       ↑       ↑       ↑       -No update
 *              SEQ0    SEQ1    SEQ0    SEQ1    SEQ0    SEQ1    SEQ0
 *                 |   /   |   /   |   /   |   /   |   /   |   /  |
 *                 SEQ2    SEQ2    SEQ2    SEQ2    SEQ2    SEQ2   |The final sequence is set to disable sequencer
 * WuptTrigger  ↑  ↑    ↑  ↑    ↑  ↑    ↑  ↑    ↑  ↑    ↑  ↑    ↑  ↑    ↑  ↑
 * Time Spend   |t1| t2 |t1| t2 |t1| t2 |t1| t2 |t1| t2 |t1| t2 |t1| t2 |t1| t2
 *                                                                 |The following triggers are ignored because sequencer is disabled
 * Wupt: Wakeup Timer
 * @endcode
 *
 * The final sequence will disable sequencer thus disable the whole measurement. It could be SEQ0 or SEQ1.  \n
 * SEQ2 will always follow SEQ0/SEQ1 to turn on ADC to sample data.                                         \n
 * SEQ0/1 and SEQ2 is managed by wakeup timer. The time delay between SEQ0/1 and SEQ
 * is set by user. Reason is that after updating DAC, signal needs some time to settle before sample it.    \n
 * In above figure, the time t1 is the delay set by user which controls where ADC starts sampling.
 * (t1+t2)*StepNumber is the total time used by ramp. It's defined by @ref RampDuration which is calculated from configured ramp parameters (scan rate etc.).
 *
 * SEQ2 commands are fixed. Function is simply turn on ADC.                                                               \n
 * SEQ0/1 is always changing its start address to update DAC with different voltage.                        \n
 * Check above figure we can see SEQ0/SEQ1 is repeatedly trigged by Wakeuptimer, if we don't change the start
 * Address of SEQ0/SEQ1, they will always update DAC with same data, thus no waveform generated.
 *
 * Considering below SEQ0 command which is similar for SEQ1 on modifying SEQxINFO register.:
 *
 * @code
 * //Total sequence command length is **7**
 * SEQ_WR(REG_AFE_AFECON, CurrentAfeCtrlReg);   //Stop ADC because current step is finished
 * SEQ_WR(REG_AFE_SYNCEXTDEVICE, 0x00000000);   //toggle pin to indicate finished sampling (can be omitted!)
 * SEQ_INT1();                                  //Generate Custom interrupt 1 to indicate new step --> MCU can process samples of previous step (averaging) and output it
 * SEQ_WR(REG_AFE_LPDACDAT0, 0x1234);           //update DAC with correct voltage
 * SEQ_WAIT(10);                                //wait 10clocks to allow DAC update
 * SEQ_WR(REG_AFE_SEQ1INFO, NextAddr|SeqLen);   //The next sequence is SEQ1, set it to correct address where stores commands.
 * SEQ_SLP();                                   //Put AFE to hibernate/sleep mode.
 * @endcode
 *
 * It will update DAC with data 0x1234, then it wait 10 clocks to allow LPDAC update.
 * The final command is to send AFE to sleep state.
 * The 6th commands here is to allow modify sequence infomation by sequencer. Above piece of commands are running by SEQ0.
 * It modify the start address of **SEQ1**. SEQ1 has same ability to update DAC data but with **different** data.
 * By the time Wakeup Timer triggers SEQ1, it will update DAC with correct data.
 *
 * The very last sequencer command (after the last step finished) is to disable sequencer.
 *
 * @code
 * SEQ_WR(REG_AFE_AFECON, CurrentAfeCtrlReg); // Stop ADC
 * SEQ_WR(REG_AFE_SYNCEXTDEVICE, 0x00000000); // toggle pin
 * SEQ_NOP();                                 // Do nothing
 * SEQ_NOP();                                 
 * SEQ_NOP();
 * SEQ_NOP();
 * SEQ_STOP();                                //Put AFE to hibernate/sleep mode.
 * @endcode
 *
 * Total SRAM is 6kB in AD594x. In normal other application, we use 2kB for sequencer and 4kB for FIFO.
 * Assume the ramp test require 128 steps, then the sequence length is 7*128 = 896, each command need 4Byte. So it costs more than 3kB SRAM.
 * When ramp test requires hundres of voltage steps(ADC samples), 2kB SRAM is far from enough. We recommend to use 4kB for sequencer
 * and 2kB for data FIFO.
 * If ramp test require more steps, then we need to update SRAM with commands dynamically (during run of ramp), use it as a ping-pong buffer.
 * That's why the SEQ0/1 section in SRAM is divided into 2 blocks (Block1 and Block2): The last DAC update command in one block triggers an interrupt
 * so that the MCU knows it has to refill the block that currently finished with new DAC codes. In the meantime, the other block is read by sequencer and executed
 * TIMING ISSUE: As for a large number of steps, the MCU needs to calculate DAC codes and write it to SRAM during the ramp test and in parallel get and process the ADC data from the current step,
 * the time for update should not be greater than  a) the SampleDelay + time when first FIFO threshold interrupt is asserted after starting of ADC and b) the time when the next step is due
 * --> Current approach that works: Use only 1 sequencer command per block to minimize DAC code calculation and SRAM write time
 * --> after each step, the MCU gets informed by interrupt 0 that the step finished so that it can calculate new code and rewrite the seq command for that block.
 *     In the meantime, the sequencer executes the step in the other block
     *
         *@{
             ** 
        */

#include "ad5940.h"
#include <stdio.h>
#include "string.h"
#include "math.h"
#include "RampTest.h"
//include the printf.h from the used arduino printf library as it forwards all printf statements to the Serial object!
//see https://www.avrfreaks.net/forum/how-use-printf-statements-external-c-file-arduino
#include "../extras/printf/printf.h"

//function prototypes that are local to this module
uint32_t DataCountCalculate(void);
void Potentiometry(void);

//global variables
volatile BoolFlag newStep = bFALSE;
volatile BoolFlag bHeapAllocated = bFALSE; //flag indicates wheter heap space was already allocated for RampVoltage variable
//int32_t *RampVoltage;
float *RampVoltage;
uint16_t volt_buf_size = 0;

/**
 * @brief The ramp application paramters.
 * @details Do not modify following default parameters. Use the function in AD5940Main.c to change it.
 *
 * */
AppRAMPCfg_Type AppRAMPCfg =
    {
        /* Common configurations for all kinds of Application. */
        .bParaChanged = bFALSE,
        .SeqStartAddr = 0,
        .MaxSeqLen = 0,
        .SeqStartAddrCal = 0,
        .MaxSeqLenCal = 0,
        /* Application related parameters */
        .LFOSCClkFreq = 32000.0,
        .SysClkFreq = 16000000.0,
        .AdcClkFreq = 16000000.0,
        .RcalVal = 10000.0,
        .ADCRefVolt = 1820.0f, /* 1.8V or 1.82V? */
        .bTestFinished = bFALSE,
        /* Describe Ramp signal */
        .RampStartVolt = +500.0f,  /* +0.5V */
        .RampPeakVolt1 = +1500.0f, /* +1.5V */
        .RampPeakVolt2 = -500.0f,  /* -0.5V */
        .Estep = 100,
        .ScanRate = 50,
        .VzeroLimitHigh = 2200.0f, /* 2.2V */
        .VzeroLimitLow = 400.0f,   /* 0.4V */
        .CycleNumber = 1,
        /* Ramp parameters that are being calculated during runtime */
        .StepNumber = 40,
        .RampDuration = 240 * 1000, /* 240s */
        .StepsToPeak1 = 10,
        .StepsToPeak2 = 10,
        .VzeroHighLevel = 2200.0f, /* 2.2V */
        .VzeroLowLevel = 400.0f,   /* 0.4V */
        /* Receive path configuration */
        .SampleDelay = 1.0f,               /* 1ms */
        .LpAmpPwrMod = LPAMPPWR_BOOST3,    /* 3mA cell current capable*/
        .LPTIARtiaSel = LPTIARTIA_20K,     /* 20 kOhm*/
        .LPTIARloadSel = LPTIARLOAD_SHORT, /* no Rload */
        .ExternalRtiaValue = 20000.0f,     /* Optional external RTIA resistore value in Ohm. */
        .AdcPgaGain = ADCPGA_1P5,          /* use factory calibrated gain of 1.5 */
        .ADCSinc3Osr = ADCSINC3OSR_4,      /* oversampling rate --> 4 is recommended, results in output data rate = 200 kSPS*/
        .ADCSinc2Osr = ADCSINC2OSR_1333,   /* as Sin3 is fed into Sinc2 and Sinc2 output store in FIFO, final data rate = 200k / 4 / 1333 = 37.5 Samples/sec */
        /* Digital related */
        .FifoThresh = 1,
        /* Priviate parameters */
        .RAMPInited = bFALSE,
        .StopRequired = bFALSE,
        .RampState = RAMP_STATE0,
        .bFirstDACSeq = bTRUE,
        .bRampOneDir = bFALSE, //set this to bTRUE for a linear sweep (LSV)!
};

/**
 * @todo add paramater check.
 * SampleDelay will limited by wakeup timer, check WUPT register value calculation equation below for reference.
 * SampleDelay > 1.0ms is acceptable.
 * RampDuration/StepNumber > 2.0ms
 * ...
 * */


/**
 * @brief This function is provided for upper controllers that want to change
 *        application parameters specially for user defined parameters.
 * @param pCfg: The pointer used to store application configuration structure pointer.
 * @return none.
*/
AD5940Err AppRAMPGetCfg(void *pCfg)
{
    if (pCfg)
    {
        *(AppRAMPCfg_Type **)pCfg = &AppRAMPCfg;
        return AD5940ERR_OK;
    }
    return AD5940ERR_PARA;
}

/**
 * @brief Control application like start, stop.
 * @param Command: The command for this application, select from below paramters
 *        - APPCTRL_START: start the measurement. Note: the ramp test need firstly call function AppRAMPInit() every time before start it.
 *        - APPCTRL_STOPNOW: Stop the measurement immediately.
 *        - APPCTRL_STOPSYNC: Stop the measuremnt when current measured data is read back.
 *        - APPCTRL_SHUTDOWN: Stop the measurement immediately and put AFE to shut down mode(turn off LP loop and enter hibernate).
 * @return none.
*/
AD5940Err AppRAMPCtrl(uint32_t Command, void *pPara)
{
    switch (Command)
    {
    case APPCTRL_START:
    {
        WUPTCfg_Type wupt_cfg;

        if (AD5940_WakeUp(10) > 10)  /* Wakeup AFE by read register, read 10 times at most */
            return AD5940ERR_WAKEUP; /* Wakeup Failed */
        if (AppRAMPCfg.RAMPInited == bFALSE)
            return AD5940ERR_APPERROR;
        /**
             *  RAMP example is special, because the sequence is dynamically generated.
             *  Before 'START' ramp test, call AppRAMPInit firstly.
             */
        if (AppRAMPCfg.RampState == RAMP_STOP)
            return AD5940ERR_APPERROR;

        /* Start it */
        wupt_cfg.WuptEn = bTRUE;
        wupt_cfg.WuptEndSeq = WUPTENDSEQ_D;
        wupt_cfg.WuptOrder[0] = SEQID_0;
        wupt_cfg.WuptOrder[1] = SEQID_2;
        wupt_cfg.WuptOrder[2] = SEQID_1;
        wupt_cfg.WuptOrder[3] = SEQID_2;
        wupt_cfg.SeqxSleepTime[SEQID_2] = 4;
        wupt_cfg.SeqxWakeupTime[SEQID_2] = (uint32_t)(AppRAMPCfg.LFOSCClkFreq * AppRAMPCfg.SampleDelay / 1000.0f) - 4 - 2;
        wupt_cfg.SeqxSleepTime[SEQID_0] = 4;
        wupt_cfg.SeqxWakeupTime[SEQID_0] = (uint32_t)(AppRAMPCfg.LFOSCClkFreq * (AppRAMPCfg.RampDuration / AppRAMPCfg.StepNumber - AppRAMPCfg.SampleDelay) / 1000.0f) - 4 - 2;
        wupt_cfg.SeqxSleepTime[SEQID_1] = wupt_cfg.SeqxSleepTime[SEQID_0];
        wupt_cfg.SeqxWakeupTime[SEQID_1] = wupt_cfg.SeqxWakeupTime[SEQID_0];
        AD5940_WUPTCfg(&wupt_cfg);
        break;
    }
    case APPCTRL_STOPNOW:
    {
        if (AD5940_WakeUp(10) > 10)  /* Wakeup AFE by read register, read 10 times at most */
            return AD5940ERR_WAKEUP; /* Wakeup Failed */
        /* Start Wupt right now */
        AD5940_WUPTCtrl(bFALSE);
        /* There is chance this operation will fail because sequencer could put AFE back 
                to hibernate mode just after waking up. Use STOPSYNC is better. */
        AD5940_WUPTCtrl(bFALSE);
        break;
    }
    case APPCTRL_STOPSYNC:
    {
        AppRAMPCfg.StopRequired = bTRUE;
        break;
    }
    case APPCTRL_SHUTDOWN:
    {
        AppRAMPCtrl(APPCTRL_STOPNOW, 0); /* Stop the measurement if it's running. */
        AD5940_ShutDownS();
    }
    break;
    default:
        break;
    }
    return AD5940ERR_OK;
}

/**
 * @brief Generate initialization sequence and write the commands to SRAM.
 * @return return error code.
*/
static AD5940Err AppRAMPSeqInitGen(void)
{
    AD5940Err error = AD5940ERR_OK;
    const uint32_t *pSeqCmd;
    uint32_t SeqLen;
    AFERefCfg_Type aferef_cfg;
    LPLoopCfg_Type lploop_cfg;
    DSPCfg_Type dsp_cfg;
    /* Start sequence generator here */
    AD5940_SEQGenCtrl(bTRUE);

    AD5940_AFECtrlS(AFECTRL_ALL, bFALSE); /* Init all to disable state */

    aferef_cfg.HpBandgapEn = bTRUE;
    aferef_cfg.Hp1V1BuffEn = bTRUE;
    aferef_cfg.Hp1V8BuffEn = bTRUE;
    aferef_cfg.Disc1V1Cap = bFALSE;
    aferef_cfg.Disc1V8Cap = bFALSE;
    aferef_cfg.Hp1V8ThemBuff = bFALSE;
    aferef_cfg.Hp1V8Ilimit = bFALSE;
    aferef_cfg.Lp1V1BuffEn = bFALSE;
    aferef_cfg.Lp1V8BuffEn = bFALSE;
    /* LP reference control - turn off them to save power*/
    aferef_cfg.LpBandgapEn = bTRUE;
    aferef_cfg.LpRefBufEn = bTRUE;
    aferef_cfg.LpRefBoostEn = bFALSE;
    AD5940_REFCfgS(&aferef_cfg);

    lploop_cfg.LpAmpCfg.LpAmpSel = LPAMP0;
    lploop_cfg.LpAmpCfg.LpAmpPwrMod = AppRAMPCfg.LpAmpPwrMod; // LPAMPPWR_BOOST3;
    lploop_cfg.LpAmpCfg.LpPaPwrEn = bTRUE;
    lploop_cfg.LpAmpCfg.LpTiaPwrEn = bTRUE;
    lploop_cfg.LpAmpCfg.LpTiaRf = LPTIARF_20K;
    lploop_cfg.LpAmpCfg.LpTiaRload = AppRAMPCfg.LPTIARloadSel;
    lploop_cfg.LpAmpCfg.LpTiaRtia = AppRAMPCfg.LPTIARtiaSel;
    if (AppRAMPCfg.LPTIARtiaSel == LPTIARTIA_OPEN)                                                        /* User want to use external RTIA */
        lploop_cfg.LpAmpCfg.LpTiaSW = LPTIASW(2) | LPTIASW(4) | LPTIASW(5) | LPTIASW(9) /*|LPTIASW(10)*/; /* SW5/9 is closed to support external RTIA resistor */
    else
        lploop_cfg.LpAmpCfg.LpTiaSW = LPTIASW(2) | LPTIASW(4) | LPTIASW(5);

    lploop_cfg.LpDacCfg.LpdacSel = LPDAC0;
    //set intial cell voltage before ramp test starts
    if (AppRAMPCfg.RampStartVolt > 0)
    {
        lploop_cfg.LpDacCfg.DacData6Bit = (uint32_t)((AppRAMPCfg.VzeroHighLevel - 200.0f) / DAC6BITVOLT_1LSB); //set WE potential to high level (max. 2.4V)
    }
    else
    {
        lploop_cfg.LpDacCfg.DacData6Bit = (uint32_t)((AppRAMPCfg.VzeroLowLevel - 200.0f) / DAC6BITVOLT_1LSB); //set WE potential to low level ( min. 0.2V)
    }
    lploop_cfg.LpDacCfg.DacData12Bit = (uint32_t)(lploop_cfg.LpDacCfg.DacData6Bit * 64 - AppRAMPCfg.RampStartVolt / DAC12BITVOLT_1LSB);
    lploop_cfg.LpDacCfg.DataRst = bFALSE;
    lploop_cfg.LpDacCfg.LpDacSW = LPDACSW_VBIAS2LPPA /*|LPDACSW_VBIAS2PIN*/ | LPDACSW_VZERO2LPTIA /*|LPDACSW_VZERO2PIN*/;
    lploop_cfg.LpDacCfg.LpDacRef = LPDACREF_2P5;
    lploop_cfg.LpDacCfg.LpDacSrc = LPDACSRC_MMR;
    lploop_cfg.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_12BIT; /* Step Vbias. Use 12bit DAC ouput */
    lploop_cfg.LpDacCfg.LpDacVzeroMux = LPDACVZERO_6BIT;  /* Base is Vzero. Use 6 bit DAC ouput */
    lploop_cfg.LpDacCfg.PowerEn = bTRUE;
    AD5940_LPLoopCfgS(&lploop_cfg);

    AD5940_StructInit(&dsp_cfg, sizeof(dsp_cfg));
    dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_LPTIA0_N;
    dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_LPTIA0_P;
    dsp_cfg.ADCBaseCfg.ADCPga = AppRAMPCfg.AdcPgaGain;

    dsp_cfg.ADCFilterCfg.ADCSinc3Osr = AppRAMPCfg.ADCSinc3Osr;
    dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ; /* ADC runs at 16MHz clock in this example, sample rate is 800kHz */
    dsp_cfg.ADCFilterCfg.BpSinc3 = bFALSE;         /* We use data from SINC3 filter */
    dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;
    dsp_cfg.ADCFilterCfg.BpNotch = bTRUE;
    dsp_cfg.ADCFilterCfg.ADCSinc2Osr = AppRAMPCfg.ADCSinc2Osr; /* only relevant if FIFO source is set to Sinc2, if Sinc3 and Sinc2 enabled, output data rate = ADCRate / ADCSinc3Osr / ADCSinc2Osr */
    dsp_cfg.ADCFilterCfg.ADCAvgNum = ADCAVGNUM_2;              /* Don't care because it's disabled */
    AD5940_DSPCfgS(&dsp_cfg);

    /* Sequence end. */
    AD5940_SEQGenInsert(SEQ_STOP()); /* Add one extra command to disable sequencer for initialization sequence because we only want it to run one time. */

    /* Stop sequence generator here */
    AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */
    error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
    if (error == AD5940ERR_OK)
    {
        AD5940_StructInit(&AppRAMPCfg.InitSeqInfo, sizeof(AppRAMPCfg.InitSeqInfo));
        if (SeqLen >= AppRAMPCfg.MaxSeqLen)
            return AD5940ERR_SEQLEN;

        AppRAMPCfg.InitSeqInfo.SeqId = SEQID_3;
        AppRAMPCfg.InitSeqInfo.SeqRamAddr = AppRAMPCfg.SeqStartAddr;
        AppRAMPCfg.InitSeqInfo.pSeqCmd = pSeqCmd;
        AppRAMPCfg.InitSeqInfo.SeqLen = SeqLen;
        AppRAMPCfg.InitSeqInfo.WriteSRAM = bTRUE;
        AD5940_SEQInfoCfg(&AppRAMPCfg.InitSeqInfo);
    }
    else
        return error; /* Error */
    return AD5940ERR_OK;
}

/**
 * @brief Generate ADC control sequence and write the commands to SRAM.
 * @return return error code.
*/
static AD5940Err AppRAMPSeqADCCtrlGen(void)
{
    AD5940Err error = AD5940ERR_OK;
    const uint32_t *pSeqCmd;
    uint32_t SeqLen;

    uint32_t WaitClks;
    ClksCalInfo_Type clks_cal;

    /* Sequence start. */
    AD5940_SEQGenCtrl(bTRUE);

    //*********** SEQUENCER COMMANDS ***********
    AD5940_SEQGpioCtrlS(AGPIO_Pin2);
    AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE);
    AD5940_SEQGenInsert(SEQ_WAIT(16 * 250)); /* wait 250us for reference power up */
    AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE);  /* Start ADC convert and DFT */
    //AD5940_SEQGpioCtrlS(0);
    //AD5940_EnterSleepS();/* Goto hibernate */
    //*********** SEQUENCER COMMANDS ***********

    /* Sequence end. */
    error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
    AD5940_SEQGenCtrl(bFALSE); /* Stop sequencer generator */

    if (error == AD5940ERR_OK)
    {
        AD5940_StructInit(&AppRAMPCfg.ADCSeqInfo, sizeof(AppRAMPCfg.ADCSeqInfo));
        if ((SeqLen + AppRAMPCfg.InitSeqInfo.SeqLen) >= AppRAMPCfg.MaxSeqLen)
            return AD5940ERR_SEQLEN;
        AppRAMPCfg.ADCSeqInfo.SeqId = SEQID_2;
        AppRAMPCfg.ADCSeqInfo.SeqRamAddr = AppRAMPCfg.InitSeqInfo.SeqRamAddr + AppRAMPCfg.InitSeqInfo.SeqLen;
        AppRAMPCfg.ADCSeqInfo.pSeqCmd = pSeqCmd;
        AppRAMPCfg.ADCSeqInfo.SeqLen = SeqLen;
        AppRAMPCfg.ADCSeqInfo.WriteSRAM = bTRUE;
        AD5940_SEQInfoCfg(&AppRAMPCfg.ADCSeqInfo);
    }
    else
        return error; /* Error */
    return AD5940ERR_OK;
}

/**
 * @brief Calculate DAC code step by step.
 * @details The calculation is based on following variables.
 *          - RampStartVolt
 *          - RampPeakVolt
 *          - VzeroStart
 *          - VzeroPeak
 *          - StepNumber
 *          Below variables must be initialzed before call this function. It's done in function @ref AppRAMPInit
 *          - RampState
 *          - CurrStepPos
 *          - bDACCodeInc
 *          - CurrRampCode
 * RAMP_STATE0: init ramp direction (sign of DAC code increment)
 * RAMP_STATE1: ramping from RampStartVolt to RampPeakVolt1
 * RAMP_STATE2: ramping from RampPeakVolt1 to RampStartVolt
 * RAMP_STATE3: ramping from RampStartVolt to RampPeakVolt2
 * RAMP_STATE4: ramping from RampPeakVolt2 to RampStartVolt 
 * RAMP_STOP: do nothing
 * @return return error code.
*/
static AD5940Err RampDacRegUpdate(uint32_t *pDACData)
{
    uint32_t VbiasCode, VzeroCode;

    if (AppRAMPCfg.bRampOneDir) //LSV
    {
        switch (AppRAMPCfg.RampState)
        {
        case RAMP_STATE0: /* init ramp direction and go to RAMP_STATE1 */
            if (AppRAMPCfg.RampStartVolt < AppRAMPCfg.RampPeakVolt1)
            {
                AppRAMPCfg.bDACCodeInc = bTRUE; //ramp up (increase ramp code)
            }
            else
            {
                AppRAMPCfg.bDACCodeInc = bFALSE; //ramp down (decrease ramp code)
            }
            //if this is not the first cycle, cell voltage is RampPeakVolt1 now --> set voltage back according to set RampStartVolt
            if (AppRAMPCfg.RampStartVolt < AppRAMPCfg.RampPeakVolt1)
            {
                AppRAMPCfg.CurrRampCode = AppRAMPCfg.RampPeakVolt1 / DAC12BITVOLT_1LSB - AppRAMPCfg.StepsToPeak1 * AppRAMPCfg.DACCodePerStep;
            }
            else
            {
                AppRAMPCfg.CurrRampCode = AppRAMPCfg.RampPeakVolt1 / DAC12BITVOLT_1LSB + AppRAMPCfg.StepsToPeak1 * AppRAMPCfg.DACCodePerStep;
            }
            //move to next state
            AppRAMPCfg.RampState = RAMP_STATE1;
            break;
        case RAMP_STATE1: /* Ramp from RampStartVolt to RampPeakVolt1 */
            //cycle is over
            if (AppRAMPCfg.CurrStepPos % AppRAMPCfg.StepNumber == (AppRAMPCfg.StepNumber - 1))
                AppRAMPCfg.RampState = RAMP_STATE0; /* Next step to come starts new cycle --> go back to STATE0 */
            // whole ramp test is over
            if (AppRAMPCfg.CurrStepPos >= AppRAMPCfg.StepNumber * AppRAMPCfg.CycleNumber)
                AppRAMPCfg.RampState = RAMP_STOP; /* Enter Stop */
            break;
        case RAMP_STOP:
            break;
        }
    }
    else //CV
    {
        switch (AppRAMPCfg.RampState)
        {
        case RAMP_STATE0: /* init ramp direction and go to RAMP_STATE1 */
            if (AppRAMPCfg.RampStartVolt < AppRAMPCfg.RampPeakVolt1)
            {
                AppRAMPCfg.bDACCodeInc = bTRUE; //ramp up (increase ramp code)
            }
            else
            {
                AppRAMPCfg.bDACCodeInc = bFALSE; //ramp down (decrease ramp code)
            }
            //move to next state
            AppRAMPCfg.RampState = RAMP_STATE1;
            break;
        case RAMP_STATE1: /* Ramp from RampStartVolt to RampPeakVolt1 */
            if (AppRAMPCfg.CurrStepPos % AppRAMPCfg.StepNumber >= AppRAMPCfg.StepsToPeak1)
            {
                AppRAMPCfg.RampState = RAMP_STATE2;                               /* Enter State2 */
                AppRAMPCfg.bDACCodeInc = AppRAMPCfg.bDACCodeInc ? bFALSE : bTRUE; //change ramp direction (ramp back to start)
            }
            break;
        case RAMP_STATE2: /* Ramp from RampPeakVolt1 to RampStartVolt */
            if (AppRAMPCfg.CurrStepPos % AppRAMPCfg.StepNumber >= (AppRAMPCfg.StepsToPeak1 * 2))
            {
                AppRAMPCfg.RampState = RAMP_STATE3; /* Enter State3 */
                if (AppRAMPCfg.RampStartVolt < AppRAMPCfg.RampPeakVolt2)
                {
                    AppRAMPCfg.bDACCodeInc = bTRUE; //ramp up (increase ramp code)
                }
                else
                {
                    AppRAMPCfg.bDACCodeInc = bFALSE; //ramp down (decrease ramp code)
                }
            }
            break;
        case RAMP_STATE3: /* Ramp from RampStartVolt to RampPeakVolt2*/
            if (AppRAMPCfg.CurrStepPos % AppRAMPCfg.StepNumber >= (AppRAMPCfg.StepsToPeak1 * 2 + AppRAMPCfg.StepsToPeak2))
            {
                AppRAMPCfg.RampState = RAMP_STATE4;                               /* Enter State4 */
                AppRAMPCfg.bDACCodeInc = AppRAMPCfg.bDACCodeInc ? bFALSE : bTRUE; //change ramp direction (ramp back to start)
            }
            break;
        case RAMP_STATE4: /* Ramp from RampPeakVolt2 to RampStartVolt */
            if (AppRAMPCfg.CurrStepPos % AppRAMPCfg.StepNumber == (AppRAMPCfg.StepNumber - 1))
                AppRAMPCfg.RampState = RAMP_STATE0; /* Next step to come starts new cycle --> go back to STATE0 */
            if (AppRAMPCfg.CurrStepPos >= AppRAMPCfg.StepNumber * AppRAMPCfg.CycleNumber)
                AppRAMPCfg.RampState = RAMP_STOP; /* Enter Stop */
            break;
        case RAMP_STOP:
            break;
        }
    }

    if (AppRAMPCfg.bDACCodeInc)
        AppRAMPCfg.CurrRampCode += AppRAMPCfg.DACCodePerStep;
    else
        AppRAMPCfg.CurrRampCode -= AppRAMPCfg.DACCodePerStep;

    if (AppRAMPCfg.CurrRampCode > 0)
    {
        AppRAMPCfg.CurrVzeroCode = (uint32_t)((AppRAMPCfg.VzeroHighLevel - 200.0f) / DAC6BITVOLT_1LSB); //set WE potential to high level (2.2V or 2.4V?)
    }
    else
    {
        AppRAMPCfg.CurrVzeroCode = (uint32_t)((AppRAMPCfg.VzeroLowLevel - 200.0f) / DAC6BITVOLT_1LSB); //set WE potential to low level (0.2V or 0.4V?)
    }

    //save the Ramp voltage of this step in the heap for later output
    // RampVoltage[AppRAMPCfg.CurrStepPos % volt_buf_size] = (int32_t)AppRAMPCfg.CurrRampCode;

    // AppRAMPCfg.CurrStepPos++;

    VzeroCode = AppRAMPCfg.CurrVzeroCode;
    VbiasCode = (uint32_t)(VzeroCode * 64 - AppRAMPCfg.CurrRampCode);

    RampVoltage[AppRAMPCfg.CurrStepPos % volt_buf_size] = VzeroCode * DAC6BITVOLT_1LSB - VbiasCode * DAC12BITVOLT_1LSB;

    AppRAMPCfg.CurrStepPos++;

    if (VbiasCode < (VzeroCode * 64))
        VbiasCode--;

    /* Truncate */
    if (VbiasCode > 4095)
    {
        VbiasCode = 4095;
#ifdef ADI_DEBUG
        ADI_Print("Warning: DAC range limit reached!");
#endif
    }
    if (VzeroCode > 63)
    {
        VzeroCode = 63;
#ifdef ADI_DEBUG
        ADI_Print("Warning: DAC range limit reached!");
#endif
    }

    *pDACData = (VzeroCode << 12) | VbiasCode;
    return AD5940ERR_OK;
}

/* Geneate sequence(s) to update DAC step by step */
/* Note: this function doesn't need sequencer generator */

/**
 * @brief Update DAC sequence in SRAM in real time.
 * @details This function generates sequences to update DAC code step by step. It's also called in interrupt
 *          function when one block of commands in SRAM has been completed. We don't use sequence generator to save memory.
 *          Check more details from documentation of this example. @ref Ramp_Test_Example
 * @return return error code
 *
 * */
static AD5940Err AppRAMPSeqDACCtrlGen(void)
{
    //#define SEQLEN_ONESTEP    4L  /* How many sequence commands are needed to update LPDAC. */
#define SEQLEN_ONESTEP 7L /*$DEBUG How many sequence commands are needed to update LPDAC. */
#define CURRBLK_BLK0 0    /* Current block is BLOCK0 */
#define CURRBLK_BLK1 1    /* Current block is BLOCK1 */
    AD5940Err error = AD5940ERR_OK;
    uint32_t BlockStartSRAMAddr;
    uint32_t DACData, SRAMAddr;
    uint32_t i;
    uint32_t StepsThisBlock;
    BoolFlag bIsFinalBlk;
    uint32_t SeqCmdBuff[SEQLEN_ONESTEP];
    uint32_t CurrentAfeCtrlReg;

    /* All below static variables are inited in below 'if' block. They are only used in this function */
    static BoolFlag bCmdForSeq0 = bTRUE;
    static uint32_t DACSeqBlk0Addr, DACSeqBlk1Addr;
    static uint32_t StepsRemainning, StepsPerBlock, DACSeqCurrBlk;

    /* Do some math calculations */
    if (AppRAMPCfg.bFirstDACSeq == bTRUE)
    {
        /* Reset bIsFirstRun at end of function. */

        /* Analog part */
        AppRAMPCfg.DACCodePerStep = (AppRAMPCfg.Estep < DAC12BITVOLT_1LSB) ? 1 : (AppRAMPCfg.Estep / DAC12BITVOLT_1LSB); //ensure at least one DAC increment

#if ALIGIN_VOLT2LSB                                                             //if defined, each step is exactly the same, otherwise it might differ by 1 LSB (truncating a float), however, the peak voltages won't be hit as exactly
        AppRAMPCfg.DACCodePerStep = (int32_t)(AppRAMPCfg.DACCodePerStep + 0.5); //round!
#endif
        if (AppRAMPCfg.DACCodePerStep < 0)
        {
            AppRAMPCfg.DACCodePerStep = -AppRAMPCfg.DACCodePerStep; /* Always positive */
        }

        if (AppRAMPCfg.bRampOneDir)
        {
            //LSV: Determine total number of steps from RampStartVolt, RampPeakVolt1 and the calculated step voltage
            AppRAMPCfg.StepNumber = (uint32_t)(fabsf((AppRAMPCfg.RampPeakVolt1 - AppRAMPCfg.RampStartVolt) / (AppRAMPCfg.DACCodePerStep * DAC12BITVOLT_1LSB)));
        }
        else
        {
            //CV: Determine total number of steps from the peak voltages and the calculated step voltage
            AppRAMPCfg.StepNumber = ((uint32_t)(fabsf(AppRAMPCfg.RampPeakVolt1 - AppRAMPCfg.RampPeakVolt2) / (AppRAMPCfg.DACCodePerStep * DAC12BITVOLT_1LSB))) * 2;
        }

        //if ramp (WE-RE) goes up initially, increase the ramp code
        if (AppRAMPCfg.RampStartVolt < AppRAMPCfg.RampPeakVolt1)
        {
            AppRAMPCfg.bDACCodeInc = bTRUE;
            //determine steps between RampStartVolt and RampPeakVolt1
            AppRAMPCfg.StepsToPeak1 = (uint32_t)((AppRAMPCfg.RampPeakVolt1 - AppRAMPCfg.RampStartVolt) / (AppRAMPCfg.DACCodePerStep * DAC12BITVOLT_1LSB));
        }
        else
        {
            AppRAMPCfg.bDACCodeInc = bFALSE;
            //determine steps between RampStartVolt and RampPeakVolt1
            AppRAMPCfg.StepsToPeak1 = (uint32_t)(-(AppRAMPCfg.RampPeakVolt1 - AppRAMPCfg.RampStartVolt) / (AppRAMPCfg.DACCodePerStep * DAC12BITVOLT_1LSB));
        }

        //determine steps between RampStartVolt and RampPeakVolt2
        AppRAMPCfg.StepsToPeak2 = AppRAMPCfg.StepNumber / 2 - AppRAMPCfg.StepsToPeak1;

        //set initial voltage according to set RampStartVolt
        if (AppRAMPCfg.RampStartVolt < AppRAMPCfg.RampPeakVolt1)
        {
            AppRAMPCfg.CurrRampCode = AppRAMPCfg.RampPeakVolt1 / DAC12BITVOLT_1LSB - AppRAMPCfg.StepsToPeak1 * AppRAMPCfg.DACCodePerStep;
        }
        else
        {
            AppRAMPCfg.CurrRampCode = AppRAMPCfg.RampPeakVolt1 / DAC12BITVOLT_1LSB + AppRAMPCfg.StepsToPeak1 * AppRAMPCfg.DACCodePerStep;
        }

        //Determine the total ramp duration in msec from the peak voltages and the set ScanRate
        if (AppRAMPCfg.bRampOneDir)
        {
            //LSV: sweep between RampPeakVolt1 and RampStartVolt
            AppRAMPCfg.RampDuration = (uint32_t)(fabsf(AppRAMPCfg.RampPeakVolt1 - AppRAMPCfg.RampStartVolt) / AppRAMPCfg.ScanRate) * 1000;
        }
        else
        {
            //CSV: sweep between RampPeakVolt1 and RampPeakVolt2
            AppRAMPCfg.RampDuration = (uint32_t)((fabsf(AppRAMPCfg.RampPeakVolt1 - AppRAMPCfg.RampPeakVolt2) / AppRAMPCfg.ScanRate) * 2) * 1000;
        }

        int32_t DACSeqLenMax;
        StepsRemainning = AppRAMPCfg.StepNumber * AppRAMPCfg.CycleNumber;
        DACSeqLenMax = (int32_t)AppRAMPCfg.MaxSeqLen - (int32_t)AppRAMPCfg.InitSeqInfo.SeqLen - (int32_t)AppRAMPCfg.ADCSeqInfo.SeqLen;
        if (DACSeqLenMax < SEQLEN_ONESTEP * 4)
            return AD5940ERR_SEQLEN;        /* No enough sequencer SRAM available */
        DACSeqLenMax -= SEQLEN_ONESTEP * 2; /* Reserve commands each block */
        //StepsPerBlock = DACSeqLenMax / SEQLEN_ONESTEP / 2;
        StepsPerBlock = 1; //make MCU update DAC code after each step to prevent timing issues ( in case update of a large number of steps is due)
        DACSeqBlk0Addr = AppRAMPCfg.ADCSeqInfo.SeqRamAddr + AppRAMPCfg.ADCSeqInfo.SeqLen;
        //DACSeqBlk1Addr = DACSeqBlk0Addr + StepsPerBlock * SEQLEN_ONESTEP;
        DACSeqBlk1Addr = DACSeqBlk0Addr + (StepsPerBlock + 1) * SEQLEN_ONESTEP; //+1 to make space for "final final command", which is used to stop sequencer
        DACSeqCurrBlk = CURRBLK_BLK0;

        //dynmically allocate memory (heap) for storing the cell voltage for output (printf)
        if (bHeapAllocated == bFALSE)
        {
            volt_buf_size = StepsPerBlock * 2 + 1;
            //RampVoltage = calloc(AppRAMPCfg.StepNumber, sizeof(int32_t));
            //RampVoltage = calloc(volt_buf_size, sizeof(int32_t));
            RampVoltage = calloc(volt_buf_size, sizeof(float));
            bHeapAllocated = bTRUE;
        }

        AppRAMPCfg.RampState = RAMP_STATE0; /* Init state to STATE0 */
        AppRAMPCfg.CurrStepPos = 0;

        bCmdForSeq0 = bTRUE; /* Start with SEQ0 */
    }

    if (StepsRemainning == 0)
        return AD5940ERR_OK; /* Done. */
    bIsFinalBlk = StepsRemainning <= StepsPerBlock ? bTRUE : bFALSE;
    if (bIsFinalBlk)
        StepsThisBlock = StepsRemainning;
    else
        StepsThisBlock = StepsPerBlock;
    StepsRemainning -= StepsThisBlock;

    BlockStartSRAMAddr = (DACSeqCurrBlk == CURRBLK_BLK0) ? DACSeqBlk0Addr : DACSeqBlk1Addr;
    SRAMAddr = BlockStartSRAMAddr;

    //Read the current state of the AFE Control register
    CurrentAfeCtrlReg = AD5940_ReadReg(REG_AFE_AFECON);
    //Clear the bits for ADC power (ADCEN) and ADC conversion to disable ADC
    CurrentAfeCtrlReg &= ~(AFECTRL_ADCPWR | AFECTRL_ADCCNV);

    for (i = 0; i < StepsThisBlock - 1; i++)
    {
        uint32_t CurrAddr = SRAMAddr;
        SRAMAddr += SEQLEN_ONESTEP; /* Jump to next sequence */
        RampDacRegUpdate(&DACData);

        /*Sequencer Commands - start*/
        SeqCmdBuff[0] = SEQ_WR(REG_AFE_AFECON, CurrentAfeCtrlReg); /* Stop ADC */
        SeqCmdBuff[1] = SEQ_WR(REG_AFE_SYNCEXTDEVICE, 0x00000000); /* toggle pin */
        SeqCmdBuff[2] = SEQ_INT1();                                /*Generate Custom interrupt 1 to indicate new step */
        SeqCmdBuff[3] = SEQ_WR(REG_AFE_LPDACDAT0, DACData);
        SeqCmdBuff[4] = SEQ_WAIT(10); /* !!!NOTE LPDAC need 10 clocks to update data. Before send AFE to sleep state, wait 10 extra clocks */
        SeqCmdBuff[5] = SEQ_WR(bCmdForSeq0 ? REG_AFE_SEQ1INFO : REG_AFE_SEQ0INFO,
                               (SRAMAddr << BITP_AFE_SEQ1INFO_ADDR) | (SEQLEN_ONESTEP << BITP_AFE_SEQ1INFO_LEN));
        SeqCmdBuff[6] = SEQ_NOP(); //SEQ_SLP();
        /*Sequencer Commands - end*/

        AD5940_SEQCmdWrite(CurrAddr, SeqCmdBuff, SEQLEN_ONESTEP);
        bCmdForSeq0 = bCmdForSeq0 ? bFALSE : bTRUE;
    }
    /* Add final DAC update */
    if (bIsFinalBlk) /* This is the final block */
    {
        uint32_t CurrAddr = SRAMAddr;
        SRAMAddr += SEQLEN_ONESTEP; /* Jump to next sequence */
        /* After update LPDAC with final data, we let sequencer to run 'final final' command, to disable sequencer.  */
        RampDacRegUpdate(&DACData);
        SeqCmdBuff[0] = SEQ_WR(REG_AFE_AFECON, CurrentAfeCtrlReg); /* Stop ADC */
        SeqCmdBuff[1] = SEQ_WR(REG_AFE_SYNCEXTDEVICE, 0x00000000); /* toggle pin */
        SeqCmdBuff[2] = SEQ_INT1();                                /*Generate Custom interrupt 1 to indicate new step */
        SeqCmdBuff[3] = SEQ_WR(REG_AFE_LPDACDAT0, DACData);
        SeqCmdBuff[4] = SEQ_WAIT(10); /* !!!NOTE LPDAC need 10 clocks to update data. Before send AFE to sleep state, wait 10 extra clocks */
        SeqCmdBuff[5] = SEQ_WR(bCmdForSeq0 ? REG_AFE_SEQ1INFO : REG_AFE_SEQ0INFO,
                               (SRAMAddr << BITP_AFE_SEQ1INFO_ADDR) | (SEQLEN_ONESTEP << BITP_AFE_SEQ1INFO_LEN));
        SeqCmdBuff[6] = SEQ_NOP(); //SEQ_SLP();
        AD5940_SEQCmdWrite(CurrAddr, SeqCmdBuff, SEQLEN_ONESTEP);
        CurrAddr += SEQLEN_ONESTEP;
        /* The final final command is to disable sequencer. */
        SeqCmdBuff[0] = SEQ_WR(REG_AFE_AFECON, CurrentAfeCtrlReg); /* Stop ADC */
        SeqCmdBuff[1] = SEQ_WR(REG_AFE_SYNCEXTDEVICE, 0x00000000); /* toggle pin */
        SeqCmdBuff[2] = SEQ_NOP();
        SeqCmdBuff[3] = SEQ_NOP(); /* Do nothing */
        SeqCmdBuff[4] = SEQ_NOP();
        SeqCmdBuff[5] = SEQ_NOP();
        SeqCmdBuff[6] = SEQ_STOP(); /* Stop sequencer. */
        /* Disable sequencer, END of sequencer interrupt is generated. */
        AD5940_SEQCmdWrite(CurrAddr, SeqCmdBuff, SEQLEN_ONESTEP);
    }
    else /* This is not the final block */
    {
        /* Jump to next block. */
        uint32_t CurrAddr = SRAMAddr;
        SRAMAddr = (DACSeqCurrBlk == CURRBLK_BLK0) ? DACSeqBlk1Addr : DACSeqBlk0Addr;
        RampDacRegUpdate(&DACData);
        SeqCmdBuff[0] = SEQ_WR(REG_AFE_AFECON, CurrentAfeCtrlReg); /* Stop ADC */
        SeqCmdBuff[1] = SEQ_WR(REG_AFE_SYNCEXTDEVICE, 0x00000000); /* toggle pin */
        SeqCmdBuff[2] = SEQ_INT1();                                /*Generate Custom interrupt 1 to indicate new step */
        SeqCmdBuff[3] = SEQ_WR(REG_AFE_LPDACDAT0, DACData);
        SeqCmdBuff[4] = SEQ_WAIT(10);
        SeqCmdBuff[5] = SEQ_WR(bCmdForSeq0 ? REG_AFE_SEQ1INFO : REG_AFE_SEQ0INFO,
                               (SRAMAddr << BITP_AFE_SEQ1INFO_ADDR) | (SEQLEN_ONESTEP << BITP_AFE_SEQ1INFO_LEN));
        SeqCmdBuff[6] = SEQ_INT0(); /* Generate Custom interrupt 0. */
        AD5940_SEQCmdWrite(CurrAddr, SeqCmdBuff, SEQLEN_ONESTEP);
        bCmdForSeq0 = bCmdForSeq0 ? bFALSE : bTRUE;
    }

    DACSeqCurrBlk = (DACSeqCurrBlk == CURRBLK_BLK0) ? CURRBLK_BLK1 : CURRBLK_BLK0; /* Switch between Block0 and block1 */
    if (AppRAMPCfg.bFirstDACSeq)
    {
        AppRAMPCfg.bFirstDACSeq = bFALSE;
        if (bIsFinalBlk == bFALSE)
        {
            /* Otherwise there is no need to init block1 sequence */
            error = AppRAMPSeqDACCtrlGen();
            if (error != AD5940ERR_OK)
                return error;
        }
        /* This is the first DAC sequence. */
        AppRAMPCfg.DACSeqInfo.SeqId = SEQID_0;
        AppRAMPCfg.DACSeqInfo.SeqLen = SEQLEN_ONESTEP;
        AppRAMPCfg.DACSeqInfo.SeqRamAddr = BlockStartSRAMAddr;
        AppRAMPCfg.DACSeqInfo.WriteSRAM = bFALSE; /* No need to write to SRAM. We already write them above. */
        AD5940_SEQInfoCfg(&AppRAMPCfg.DACSeqInfo);
    }
    return AD5940ERR_OK;
}

/**
 * @brief Calibrate ADC gain and offset for given PGA setting.
 * @details This function will do calibration using parameters stored in @ref AppRAMPCfg structure.
 * @return return error code.
*/
static AD5940Err AppRAMPADCPgaCal(void)
{
    ADCPGACal_Type pga_cal;

    //"If the PGA needs to be calibrated for a gain of 9 the PGA gain of 1 must be calibrated first"

    /* Calibrate ADC PGA(offset and gain)  for PGA = 1 */
    pga_cal.AdcClkFreq = AppRAMPCfg.AdcClkFreq;
    pga_cal.SysClkFreq = AppRAMPCfg.SysClkFreq;
    pga_cal.ADCPga = ADCPGA_1;
    pga_cal.ADCSinc2Osr = ADCSINC2OSR_1333; /* 800kSPS/4/1333 = 150Hz,  T = 6.67ms*/
    pga_cal.ADCSinc3Osr = ADCSINC3OSR_4;
    pga_cal.TimeOut10us = 10 * 100; /* 10ms max */
    pga_cal.VRef1p82 = AppRAMPCfg.ADCRefVolt;
    pga_cal.VRef1p11 = 1.107f;                  /* measure it on VBIAS_CAP pin (pin 31 - C6 on AD5941 FeatherWing Rev1 Board) */
    pga_cal.PGACalType = PGACALTYPE_OFFSETGAIN; /* Calibrate Offset and Gain errors */
    AD5940_ADCPGACal(&pga_cal);

    /* Calibrate Offset and Gain for PGA = application gain */
    pga_cal.ADCPga = AppRAMPCfg.AdcPgaGain;
    AD5940_ADCPGACal(&pga_cal);

    // /* Calibrate Offset and Gain for PGA = 1.5 */
    // pga_cal.ADCPga = ADCPGA_1P5;
    // AD5940_ADCPGACal(&pga_cal);
    // /* Calibrate Offset and Gain for PGA = 2 */
    // pga_cal.ADCPga = ADCPGA_2;
    // AD5940_ADCPGACal(&pga_cal);
    // /* Calibrate Offset and Gain for PGA = 4 */
    // pga_cal.ADCPga = ADCPGA_4;
    // AD5940_ADCPGACal(&pga_cal);
    // /* Calibrate Offset and Gain for PGA = 9 */
    // pga_cal.ADCPga = ADCPGA_9;
    // AD5940_ADCPGACal(&pga_cal);

#ifdef ADI_DEBUG
    ADI_Print("ADC calibrated.\n");
#endif
    return AD5940ERR_OK;
}

/**
 * @brief Calibrate LPTIA internal RTIA resistor(s).
 * @details This function will do calibration using parameters stored in @ref AppRAMPCfg structure.
 * @return return error code.
*/
static AD5940Err AppRAMPRtiaCal(void)
{
    fImpPol_Type RtiaCalValue; /* Calibration result */
    LPRTIACal_Type lprtia_cal;
    AD5940_StructInit(&lprtia_cal, sizeof(lprtia_cal));

    lprtia_cal.LpAmpSel = LPAMP0;
    lprtia_cal.bPolarResult = bTRUE; /* Magnitude + Phase */
    lprtia_cal.AdcClkFreq = AppRAMPCfg.AdcClkFreq;
    lprtia_cal.SysClkFreq = AppRAMPCfg.SysClkFreq;
    lprtia_cal.ADCSinc3Osr = ADCSINC3OSR_4;
    lprtia_cal.ADCSinc2Osr = ADCSINC2OSR_22; /* Use SINC2 data as DFT data source */
    lprtia_cal.DftCfg.DftNum = DFTNUM_2048;  /* Maximum DFT number */
    lprtia_cal.DftCfg.DftSrc = DFTSRC_SINC2NOTCH;
    lprtia_cal.DftCfg.HanWinEn = bTRUE;
    lprtia_cal.fFreq = AppRAMPCfg.AdcClkFreq / 4 / 22 / 2048 * 3; /* Sample 3 period of signal, 13.317Hz here. Do not use DC method, because it needs ADC/PGA calibrated firstly(but it's faster) */
    lprtia_cal.fRcal = AppRAMPCfg.RcalVal;
    lprtia_cal.LpTiaRtia = AppRAMPCfg.LPTIARtiaSel;
    lprtia_cal.LpAmpPwrMod = AppRAMPCfg.LpAmpPwrMod; //LPAMPPWR_NORM;
    lprtia_cal.bWithCtia = bFALSE;
    AD5940_LPRtiaCal(&lprtia_cal, &RtiaCalValue);
    AppRAMPCfg.RtiaValue = RtiaCalValue;
    //printf("Rtia,%f,%f\n", RtiaCalValue.Magnitude, RtiaCalValue.Phase);
#ifdef ADI_DEBUG
    //ADI_Print("Rtia,%d,%d\n", (int) RtiaCalValue.Magnitude, RtiaCalValue.Phase);
    ADI_Print("Calibrated Rtia - Magnitude = %d.", ((int)RtiaCalValue.Magnitude));
    ADI_Print("%d Ohm\n", (int)((RtiaCalValue.Magnitude * 1000.0 - ((int)RtiaCalValue.Magnitude * 1000)) + 0.0005));
#endif
    return AD5940ERR_OK;
}

/**
 * @brief Calibrate LPTIA offset.
 * @details This function will do calibration using parameters stored in @ref AppRAMPCfg structure.
 * @return return error code.
*/
static AD5940Err AppRAMPLPTIAOffsetCal(void)
{
    AD5940Err error = AD5940ERR_OK;
    LPTIAOffsetCal_Type lptiaoffset_cal;
    uint32_t lpdac6bit, lpdac12bit;
    /* LP Loop  */
    lpdac6bit = (uint32_t)((AppRAMPCfg.VzeroHighLevel - 200) / DAC6BITVOLT_1LSB);
    if (lpdac6bit > 0x3f)
        lpdac12bit = 0x3f;
    //lpdac12bit = (uint32_t)(lpdac6bit * 64 + AppRAMPCfg.SensorBias/DAC12BITVOLT_1LSB);
    lpdac12bit = lpdac6bit * 64;
    if (lpdac12bit > 0xfff)
        lpdac12bit = 0xfff;
    if (lpdac12bit < (lpdac6bit * 64))
        lpdac12bit--;
    /* calibration LPTIA offset */
    lptiaoffset_cal.AdcClkFreq = 16e6;
    lptiaoffset_cal.SysClkFreq = 16e6;
    lptiaoffset_cal.ADCSinc2Osr = ADCSINC2OSR_1333;
    lptiaoffset_cal.ADCSinc3Osr = ADCSINC3OSR_4;
    lptiaoffset_cal.ADCPga = AppRAMPCfg.AdcPgaGain;
    lptiaoffset_cal.DacData6Bit = lpdac6bit;
    lptiaoffset_cal.DacData12Bit = lpdac12bit;
    lptiaoffset_cal.LpDacVzeroMux = LPDACVZERO_6BIT;
    lptiaoffset_cal.LpAmpPwrMod = AppRAMPCfg.LpAmpPwrMod;
    lptiaoffset_cal.LpTiaRtia = AppRAMPCfg.LPTIARtiaSel;
    lptiaoffset_cal.LpTiaSW = LPTIASW(13) | LPTIASW(12) | LPTIASW(8) | LPTIASW(5);
    lptiaoffset_cal.SettleTime10us = 5000 * 100; /* 1000ms, the time needed for RTIA//CTIA settlling. */
    lptiaoffset_cal.TimeOut10us = 10 * 100;      /* 10ms. */
    error = AD5940_LPTIAOffsetCal(&lptiaoffset_cal);
    if (error != AD5940ERR_OK)
    {
#ifdef ADI_DEBUG
        ADI_Print("ERR: LPTIA Offset calibration error! %d\n", error);
#endif
        return error;
    }
    return AD5940ERR_OK;
}

/**
 * @brief Initialize the ramp test. Call this functions every time before start ramp test.
 * @param pBuffer: the buffer for sequencer generator. Only need to provide it for the first time.
 * @param BufferSize: The buffer size start from pBuffer.
 * @return return error code.
*/
AD5940Err AppRAMPInit(uint32_t *pBuffer, uint32_t BufferSize)
{
    AD5940Err error = AD5940ERR_OK;
    FIFOCfg_Type fifo_cfg;
    SEQCfg_Type seq_cfg;

    if (AppRAMPCfg.VzeroLimitHigh < AppRAMPCfg.VzeroLimitLow)
        return AD5940ERR_PARA;

#ifdef FIX_WE_POT
    //determine the range in which we can sweep the cell potential
    float range = (AppRAMPCfg.VzeroLimitHigh - AppRAMPCfg.VzeroLimitLow);
    float Peak1;
    float Peak2;

    //determine the peak voltages depending on voltammetric method
    if (AppRAMPCfg.bRampOneDir == bTRUE)
    {
        //LSV
        Peak1 = AppRAMPCfg.RampStartVolt;
        Peak2 = AppRAMPCfg.RampPeakVolt1;
    }
    else
    {
        //CV
        Peak1 = AppRAMPCfg.RampPeakVolt1;
        Peak2 = AppRAMPCfg.RampPeakVolt2;
    }

    //check if the difference between the peak voltages is compatible with the achievable range, a headroom of at least DAC6BITVOLT_1LSB is needed
    if ((fabsf(Peak1 - Peak2) + DAC6BITVOLT_1LSB) > range)
    {
#ifdef ADI_DEBUG
        ADI_Print("Peak Voltages out of range!");
#endif
        return AD5940ERR_PARA;
    }
    //set Vzero to mid-level -> allowed range: +/-(VzeroHighLevel - VzeroLowLevel) / 2
    AppRAMPCfg.VzeroHighLevel = (AppRAMPCfg.VzeroLimitHigh - AppRAMPCfg.VzeroLimitLow) / 2 + AppRAMPCfg.VzeroLimitLow;
    AppRAMPCfg.VzeroLowLevel = AppRAMPCfg.VzeroHighLevel;
    //the +/-range
    range /= 2;
    //in case one of the peak voltages is higher than +/-(VzeroHighLevel - VzeroLowLevel) / 2
    //Adding/ Substracting DAC6BITVOLT_1LSB needed to prevent negative 12-Bit DAC codes in RampDacRegUpdate() where VzeroCode gets trunkated
    if (Peak1 > range)
        AppRAMPCfg.VzeroHighLevel += Peak1 - range + DAC6BITVOLT_1LSB; //increase WE potential in case the peak is positive and above the +range
    else if (Peak1 < -range)
        AppRAMPCfg.VzeroHighLevel -= fabsf(Peak1) - range - DAC6BITVOLT_1LSB; //decreaseWE potential in case the peak is negative and below the -range
    else if (Peak2 > range)
        AppRAMPCfg.VzeroHighLevel += Peak2 - range + DAC6BITVOLT_1LSB; //increase WE potential in case the peak is positive and above the +range
    else if (Peak2 < -range)
        AppRAMPCfg.VzeroHighLevel -= fabsf(Peak2) - range - DAC6BITVOLT_1LSB; //decreaseWE potential in case the peak is negative and below the -range
    //WE potential not allowed to switch anymore -> set both levels to same value
    AppRAMPCfg.VzeroLowLevel = AppRAMPCfg.VzeroHighLevel;
#else
    //allow the Vzero level to switch between the limit values
    AppRAMPCfg.VzeroHighLevel = AppRAMPCfg.VzeroLimitHigh;
    AppRAMPCfg.VzeroLowLevel = AppRAMPCfg.VzeroLimitLow;
#endif

    if (AD5940_WakeUp(10) > 10)  /* Wakeup AFE by read register, read 10 times at most */
        return AD5940ERR_WAKEUP; /* Wakeup Failed */

    /* Configure sequencer and stop it */
    seq_cfg.SeqMemSize = SEQMEMSIZE_4KB;
    seq_cfg.SeqBreakEn = bFALSE;
    seq_cfg.SeqIgnoreEn = bFALSE;
    seq_cfg.SeqCntCRCClr = bTRUE;
    seq_cfg.SeqEnable = bFALSE;
    seq_cfg.SeqWrTimer = 0;
    AD5940_SEQCfg(&seq_cfg);

    /* Initialize sequencer generator */
    if ((AppRAMPCfg.RAMPInited == bFALSE) ||
        (AppRAMPCfg.bParaChanged == bTRUE))
    {
        if (pBuffer == 0)
            return AD5940ERR_PARA;
        if (BufferSize == 0)
            return AD5940ERR_PARA;

        //calibrate ADC
        //AppRAMPADCPgaCal();

        //calibrate LPTIA offset CAUTION: disconnect sensor!!!
        //AppRAMPLPTIAOffsetCal();

        if (AppRAMPCfg.LPTIARtiaSel == LPTIARTIA_OPEN) /* Internal RTIA is opened. User wants to use external RTIA resistor */
        {
            AppRAMPCfg.RtiaValue.Magnitude = AppRAMPCfg.ExternalRtiaValue;
            AppRAMPCfg.RtiaValue.Phase = 0;
        }
        else
            AppRAMPRtiaCal();

#ifdef ADI_DEBUG
        int PgaGain[] = {10, 15, 20, 40, 90}; //factor 10
        float range = 0;
        float res = 0;
        char floatBuffer[64];
        //current range
        //check different ADC input voltage ranges for different PGA settings
        if (PgaGain[AppRAMPCfg.AdcPgaGain] <= 15) //PGA = 1, 1.5
            range = 900.0 / AppRAMPCfg.RtiaValue.Magnitude * 1000;
        else if (PgaGain[AppRAMPCfg.AdcPgaGain] == 20) //PGA = 2
            range = 600.0 / AppRAMPCfg.RtiaValue.Magnitude * 1000;
        else if (PgaGain[AppRAMPCfg.AdcPgaGain] == 40) //PGA = 4
            range = 300.0 / AppRAMPCfg.RtiaValue.Magnitude * 1000;
        else if (PgaGain[AppRAMPCfg.AdcPgaGain] == 90) //PGA = 9
            range = 133.0 / AppRAMPCfg.RtiaValue.Magnitude * 1000;
        //convert float to char
        snprintf(floatBuffer, sizeof floatBuffer, "%f", range);
        ADI_Print("Current range: +/- %s uA\n", floatBuffer);

        //output the resolution of current measurement based on the calibrated RTIA
        if (PgaGain[AppRAMPCfg.AdcPgaGain] == 15)
            //different reference voltage for PGA 1.5 (see datasheet page 54)
            res = 1835 / (32768 * AppRAMPCfg.RtiaValue.Magnitude * (PgaGain[AppRAMPCfg.AdcPgaGain] / 10)) * 1e6;
        else
            res = AppRAMPCfg.ADCRefVolt / (32768 * AppRAMPCfg.RtiaValue.Magnitude * (PgaGain[AppRAMPCfg.AdcPgaGain] / 10)) * 1e6;
        //convert float to char
        snprintf(floatBuffer, sizeof floatBuffer, "%f", res);
        ADI_Print("Current resolution: %s nA\n", floatBuffer);
#endif

        //measure the equilibrium potential [and change the start potential accordingly]
        //Potentiometry();

        AppRAMPCfg.RAMPInited = bFALSE;
        AD5940_SEQGenInit(pBuffer, BufferSize);
        /* Generate sequence and write them to SRAM start from address AppRAMPCfg.SeqStartAddr */
        error = AppRAMPSeqInitGen(); /* Application initialization sequence */
        if (error != AD5940ERR_OK)
            return error;
        error = AppRAMPSeqADCCtrlGen(); /* ADC control sequence */
        if (error != AD5940ERR_OK)
            return error;
        AppRAMPCfg.bParaChanged = bFALSE; /* Clear this flag as we already implemented the new configuration */
    }

    /* Reconfigure FIFO, The Rtia calibration function may generate data that stored to FIFO */
    AD5940_FIFOCtrlS(FIFOSRC_SINC3, bFALSE); /* Disable FIFO firstly */
    fifo_cfg.FIFOEn = bTRUE;
    //fifo_cfg.FIFOSrc = FIFOSRC_SINC3;
    fifo_cfg.FIFOSrc = FIFOSRC_SINC2NOTCH;
    fifo_cfg.FIFOThresh = AppRAMPCfg.FifoThresh; /* Change FIFO paramters */
    fifo_cfg.FIFOMode = FIFOMODE_FIFO;
    fifo_cfg.FIFOSize = FIFOSIZE_2KB;
    AD5940_FIFOCfg(&fifo_cfg);

    /* Clear all interrupts */
    AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
    /* Generate DAC sequence */
    AppRAMPCfg.bFirstDACSeq = bTRUE;
    error = AppRAMPSeqDACCtrlGen();
    if (error != AD5940ERR_OK)
        return error;

#ifdef ADI_DEBUG
    ADI_Print("StepNumber: %u\n", AppRAMPCfg.StepNumber);
    ADI_Print("RampDuration = %u ms\n", AppRAMPCfg.RampDuration);
    int Sinc3OSR[] = {2, 4, 5};
    int Sinc2OSR[] = {22, 44, 89, 178, 267, 533, 640, 667, 800, 889, 1067, 1333};
    //#samples per step = Sampling freq * (step duration - sample SampleDelay)
    ADI_Print("Estimated #samples per step (rounded): %u\n", (int)(800000.0 / Sinc3OSR[AppRAMPCfg.ADCSinc3Osr] / Sinc2OSR[AppRAMPCfg.ADCSinc2Osr] * (AppRAMPCfg.RampDuration / AppRAMPCfg.StepNumber - AppRAMPCfg.SampleDelay) / 1000 + 0.5));
#endif

    /* Configure sequence info. */
    AppRAMPCfg.InitSeqInfo.WriteSRAM = bFALSE;
    AD5940_SEQInfoCfg(&AppRAMPCfg.InitSeqInfo);

    AD5940_SEQCtrlS(bTRUE);                          /* Enable sequencer */
    AD5940_SEQMmrTrig(AppRAMPCfg.InitSeqInfo.SeqId); // run the init sequence?!
    while (AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_ENDSEQ) == bFALSE)
        ;
    AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);

    AppRAMPCfg.ADCSeqInfo.WriteSRAM = bFALSE;
    AD5940_SEQInfoCfg(&AppRAMPCfg.ADCSeqInfo);

    AppRAMPCfg.DACSeqInfo.WriteSRAM = bFALSE;
    AD5940_SEQInfoCfg(&AppRAMPCfg.DACSeqInfo);

    AD5940_SEQCtrlS(bFALSE);
    AD5940_WriteReg(REG_AFE_SEQCNT, 0);
    AD5940_SEQCtrlS(bTRUE); /* Enable sequencer, and wait for trigger */
    AD5940_ClrMCUIntFlag(); /* Clear interrupt flag generated before */

    AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ); /* Set to low power mode */

    AppRAMPCfg.RAMPInited = bTRUE; /* RAMP application has been initialized. */
    return AD5940ERR_OK;
}

/**
 * @brief This function is called in ISR when AFE has been wakeup and we can access registers.
 * @param pData: the buffer points to data read back from FIFO. Not needed for this application-RAMP
 * @param pDataCount: The data count in pData buffer.
 * @return return error code.
*/
static int32_t AppRAMPRegModify(int32_t *const pData, uint32_t *pDataCount)
{
    if (AppRAMPCfg.StopRequired == bTRUE)
    {
        AD5940_WUPTCtrl(bFALSE);
        return AD5940ERR_OK;
    }
    return AD5940ERR_OK;
}

/**
 * @brief Sum up samples from FIFO, if a step is finished: calculate average ADC code and determine corresponding current value
 * @param pData: the buffer points to data read back from FIFO. It is used to store the step voltage and corresponding current result
 * @param pDataCount: The data count in pData buffer. During the step this should maximally be FIFOThresh (smaller if step duration is short). At the end of a step it likely will be smaller
 * @return return newMean: 1 if a step finished (average current is available for output), otherwise 0
*/
static int32_t AppRAMPDataProcess(int32_t *const pData, uint32_t *pDataCount)
{
    static uint32_t sample_cnt = 0; //counts the number of samples in each step
    static uint64_t SumSamples = 0; //hold the sums of samples (ADC codes) during one voltage plateau to calc. average
    static uint32_t step_cnt = 0;   //counts number of steps during one ramp

    uint8_t newMean = 0; //indicates wether an average current value together with its corresponing voltage is available. 0 if no new step occured.
    uint32_t i, datacount;
    datacount = *pDataCount;
    float *pOut = (float *)pData; //use pData to output voltage and current
    float temp;
    uint32_t AverageCode = 0; //the average ADC code in one step

    for (i = 0; i < datacount; i++)
    {

        SumSamples += pData[i] & 0xffff;
        sample_cnt++;
    }

    //the step is finished-> output sample mean
    if (newStep == bTRUE)
    {
        newStep = bFALSE; //reset new step indicator flag

        if (sample_cnt > 0) //it is not the initial step (new step interrupt right after first step-> no data in FIFO!)
        {

#ifdef ADI_DEBUG
            //print the number of samples from the finished step
            ADI_Print("%u ", sample_cnt);
#endif
            //output the corresponding cell voltage of the old step
            //pOut[newMean] = RampVoltage[step_cnt % volt_buf_size] * DAC12BITVOLT_1LSB;
            pOut[newMean] = RampVoltage[step_cnt % volt_buf_size];

            //output the mean or indicate that a mean is ready for a whole step  (global array / pointer to array? --> use first entry in pOut!)
            //calculate mean ADC code of old samples
            AverageCode = (uint32_t)((float)SumSamples / sample_cnt + 0.5); //round to one LSB
            //turn it into a voltage
            temp = AD5940_ADCCode2Volt(AverageCode, AppRAMPCfg.AdcPgaGain, AppRAMPCfg.ADCRefVolt);
            //calculate current: no negative sign, because oxidation (-> electrons go into SE -> current out of SE --> ADC measures positive voltage across RTIA) should have positive current
            pOut[newMean + 1] = temp / AppRAMPCfg.RtiaValue.Magnitude * 1e3f; /* Result unit is uA. */

            newMean += 2; //indicate that a step finished --> mean is ready
            step_cnt++;

            SumSamples = 0;
            sample_cnt = 0;
        }
    }

    //reset the static variables after the last step of one ramp (important if we start multipl measurements via cloud)
    if (step_cnt == AppRAMPCfg.StepNumber * AppRAMPCfg.CycleNumber)
    {
        sample_cnt = 0;
        step_cnt = 0;
        SumSamples = 0;
    }

    //return number of new sample means (and corresponding voltages) in the aquired FIFO data: 1 or 0
    return (newMean / 2);
    //in Main: if return value > 0 --> take the first newMean elements of pOut and print it/ save it in Average-Array
}

/**
 * @brief The interrupt service routine for RAMP test.
 * @param pBuff: The buffer provides by host, used to store data read back from FIFO.
 * @param pCount: The available buffer size starts from pBuff.
 * @return return error code.
*/
AD5940Err AppRAMPISR(void *pBuff, uint32_t *pCount)
{
    uint32_t BuffCount;
    uint32_t FifoCnt;
    BuffCount = *pCount;
    uint32_t IntFlag;

    if (AD5940_WakeUp(10) > 10)  /* Wakeup AFE by read register, read 10 times at most */
        return AD5940ERR_WAKEUP; /* Wakeup Failed */
    AD5940_SleepKeyCtrlS(SLPKEY_LOCK);
    *pCount = 0;
    IntFlag = AD5940_INTCGetFlag(AFEINTC_0);
    //repeat ISR routine as long as there are interrupts to handle, cause some may be triggered during processing of another interrupt
    while (IntFlag != 0)
    {
        if (IntFlag & AFEINTSRC_CUSTOMINT1) //new voltage step, handle before new DAC code write so that correct voltage gets output
        {
            newStep = bTRUE;           //set flag to output finished step mean

            //get the remaining data of the finished step
            FifoCnt = AD5940_FIFOGetCnt();
            AD5940_FIFORd((uint32_t *)pBuff, FifoCnt);
            AD5940_INTCClrFlag(AFEINTSRC_CUSTOMINT1);
            /* Process data, get the number of finished steps (= number of calculated means) within the current FIFO data set */
            *pCount = AppRAMPDataProcess((int32_t *)pBuff, &FifoCnt);
        }

        if (IntFlag & AFEINTSRC_CUSTOMINT0) /* High priority: End of one block with DAC commands --> rewrite new DAC codes to SRAM block that finished*/
        {
            AD5940Err error;
            AD5940_INTCClrFlag(AFEINTSRC_CUSTOMINT0);
            error = AppRAMPSeqDACCtrlGen();
            if (error != AD5940ERR_OK)
                return error;
            AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
            //AD5940_EnterSleepS(); /* If there is need to do AFE re-configure, do it here when AFE is in active state */
        }

        if (IntFlag & AFEINTSRC_DATAFIFOTHRESH)
        {
            FifoCnt = AD5940_FIFOGetCnt();

            if (FifoCnt > BuffCount)
            {
                ///@todo buffer is limited.
            }
            AD5940_FIFORd((uint32_t *)pBuff, FifoCnt);
            AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
            AppRAMPRegModify(pBuff, &FifoCnt); /* Stop the Wakeup Timer if Stop is required */
            AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
            //AD5940_EnterSleepS();
            /* Process data, get the number of finished steps (= number of calculated means) within the current FIFO data set */
            *pCount = AppRAMPDataProcess((int32_t *)pBuff, &FifoCnt);
            //*pCount = FifoCnt;
            //return 0;
        }
        if (IntFlag & AFEINTSRC_GPT1INT_TRYBRK)
        {
            AD5940_AGPIOClr(AGPIO_Pin1); //turn on LED
#ifdef ADI_DEBUG
            ADI_Print("Sequences timing overlap error\n");
#endif
        }
        if (IntFlag & AFEINTSRC_CMDFIFOOF)
        {
            AD5940_AGPIOClr(AGPIO_Pin1); //turn on LED
#ifdef ADI_DEBUG
            ADI_Print("FIFO overflow\n");
#endif
        }

        if (IntFlag & AFEINTSRC_ENDSEQ)
        {
            AFERefCfg_Type aferef_cfg;
            ADCFilterCfg_Type adcfilter_cfg;
            //get remaining data from last step
            FifoCnt = AD5940_FIFOGetCnt();
            AD5940_FIFORd((uint32_t *)pBuff, FifoCnt);
            AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);
            newStep = bTRUE; //set the newStep flag to make the AppRAMPDataProcess() output the mean of the previous samples!
            /* Process data from last step, get the number of finished steps (= number of calculated means) within the current FIFO data set */
            *pCount = AppRAMPDataProcess((int32_t *)pBuff, &FifoCnt); //pBuff contains data value that gets added to the next step (which does not exist -> don't care whats in pBuff)
            //*pCount = FifoCnt;
            AppRAMPCtrl(APPCTRL_STOPNOW, 0); /* Stop the Wakeup Timer. */

            /* Reset variables so measurement can be restarted*/
            //free(RampVoltage);
            AppRAMPCfg.bTestFinished = bTRUE;
            AppRAMPCfg.RampState = RAMP_STATE0;
            AppRAMPCfg.bFirstDACSeq = bTRUE;
            AppRAMPCfg.bDACCodeInc = bTRUE;

            //disable AFE (important for LSV: this prevents that WE stays at RampPeakVolt1 at the end)
            AD5940_ShutDownS();
        }

        IntFlag = AD5940_INTCGetFlag(AFEINTC_0);
    }

    return 0;
}


/**
 * @brief measures the equilibrium potential btw WE and RE (Cell voltage)
 * @return none
*/
void Potentiometry(void)
{
    DSPCfg_Type dsp_cfg;
    FIFOCfg_Type fifo_cfg;
    uint32_t FifoCnt;
    uint32_t ADCBuff[512];
    float Cell_volt; //cell potential, mV
    char floatBuffer[64];
    uint64_t SampleMean = 0;

    LPLoopCfg_Type lploop_cfg;

    AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ); /* Set to low power mode */

    //LPAMP needs to be powered on for the ADC to measure the negative input, disable the rest and set switches so that sensor does not see any voltages
    lploop_cfg.LpAmpCfg.LpAmpSel = LPAMP0;
    lploop_cfg.LpAmpCfg.LpAmpPwrMod = AppRAMPCfg.LpAmpPwrMod; //LPAMPPWR_BOOST3;
    lploop_cfg.LpAmpCfg.LpPaPwrEn = bFALSE;
    lploop_cfg.LpAmpCfg.LpTiaPwrEn = bTRUE;
    lploop_cfg.LpAmpCfg.LpTiaRf = LPTIARF_20K;
    lploop_cfg.LpAmpCfg.LpTiaRload = LPTIARLOAD_SHORT;
    lploop_cfg.LpAmpCfg.LpTiaRtia = LPTIARTIA_OPEN;
    //leave SW2 and SW4 open to prevent controlling the CE/ RE pins
    lploop_cfg.LpAmpCfg.LpTiaSW = LPTIASW(5) | LPTIASW(12) | LPTIASW(13);

    lploop_cfg.LpDacCfg.LpdacSel = LPDAC0;
    lploop_cfg.LpDacCfg.DacData6Bit = (uint32_t)((500.0 - 200.0) / DAC6BITVOLT_1LSB);    //Don't care, not connected to sensor.
    lploop_cfg.LpDacCfg.DacData12Bit = (uint32_t)((1300.0 - 200.0) / DAC12BITVOLT_1LSB); //Don't care, not connected to sensor.
    lploop_cfg.LpDacCfg.DataRst = bFALSE;
    lploop_cfg.LpDacCfg.LpDacSW = LPDACSW_VBIAS2PIN | LPDACSW_VZERO2PIN;
    lploop_cfg.LpDacCfg.LpDacRef = LPDACREF_2P5;
    lploop_cfg.LpDacCfg.LpDacSrc = LPDACSRC_MMR;
    lploop_cfg.LpDacCfg.LpDacVbiasMux = LPDACVBIAS_12BIT; /* Step Vbias. Use 12bit DAC ouput */
    lploop_cfg.LpDacCfg.LpDacVzeroMux = LPDACVZERO_6BIT;  /* Base is Vzero. Use 6 bit DAC ouput */
    lploop_cfg.LpDacCfg.PowerEn = bFALSE;
    AD5940_LPLoopCfgS(&lploop_cfg);

    //init struct with zeroes
    AD5940_StructInit(&dsp_cfg, sizeof(dsp_cfg));

    //***Measure -Cell_volt = RE0 - SE0***
    //ADC setting
    dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_LPTIA0_N; //LPTIA0 negative input node --> routes to SE0 (see Fig 23 in datasheet)
    dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_VRE0;     //Pin RE0
    dsp_cfg.ADCBaseCfg.ADCPga = AppRAMPCfg.AdcPgaGain;

    //Initialize ADC filters ADCRawData-->SINC3-->SINC2+NOTCH-->StatisticBlock
    dsp_cfg.ADCFilterCfg.ADCSinc3Osr = ADCSINC3OSR_2;
    dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ; /* ADC runs at 16MHz clock in this example, sample rate is 800kHz */
    dsp_cfg.ADCFilterCfg.BpSinc3 = bFALSE;         /* We use data from SINC3 filter */
    dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;
    dsp_cfg.ADCFilterCfg.BpNotch = bFALSE;               //use notch filter for simultaneous 50/60 Hz mains filtering
    dsp_cfg.ADCFilterCfg.ADCSinc2Osr = ADCSINC2OSR_1333; /* only relevant if FIFO source is set to Sinc2, if Sinc3 and Sinc2 enabled, output data rate = ADCRate / ADCSinc3Osr / ADCSinc2Osr */
    dsp_cfg.ADCFilterCfg.ADCAvgNum = ADCAVGNUM_2;        /* Don't care about it. Average function is only used for DFT */

    //Statistic block settings
    //Statistic block receive data from SINC2+Notch block. Note the diagram in datasheet page 51 PrM. The SINC3 can be bypassed optionally. SINC2 cannot be bypassed.
    dsp_cfg.StatCfg.StatDev = STATDEV_1; /* Not used. */
    dsp_cfg.StatCfg.StatEnable = bFALSE;
    dsp_cfg.StatCfg.StatSample = STATSAMPLE_128; /* Sample 128 points and calculate mean. */

    AD5940_DSPCfgS(&dsp_cfg);

    //configure FIFO
    fifo_cfg.FIFOEn = bTRUE;
    fifo_cfg.FIFOMode = FIFOMODE_FIFO;
    fifo_cfg.FIFOSize = FIFOSIZE_2KB;
    fifo_cfg.FIFOSrc = FIFOSRC_SINC2NOTCH; // FIFOSRC_MEAN; //use output of statistic block - mean as data source
    fifo_cfg.FIFOThresh = 128;
    AD5940_FIFOCfg(&fifo_cfg);

    AD5940_INTCClrFlag(AFEINTSRC_ALLINT); /* Clear all interrupts */
    AD5940_ClrMCUIntFlag();               /* Clear the MCU interrupt flag which will be set in ISR. */

    //while(1)-loop for testing purposes- remove it for real application!
    while (1)
    {
        //start ADC
        AD5940_AFECtrlS(AFECTRL_ADCPWR | AFECTRL_SINC2NOTCH, bTRUE);
        AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE);

        while (!AD5940_GetMCUIntFlag())
        {
            //wait until interrupt
        }

        if (AD5940_INTCGetFlag(AFEINTC_0) & AFEINTSRC_DATAFIFOTHRESH)
        {
            //disable ADC
            AD5940_AFECtrlS(AFECTRL_ADCCNV, bFALSE);
            AD5940_AFECtrlS(AFECTRL_ADCPWR | AFECTRL_SINC2NOTCH, bFALSE);

            FifoCnt = AD5940_FIFOGetCnt();
            //AD5940_FIFORd((uint32_t *)ADCBuff, 1);
            SampleMean = 0; //reset sample mean
            AD5940_FIFORd((uint32_t *)ADCBuff, FifoCnt);
            AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
            AD5940_ClrMCUIntFlag(); /* Clear the MCU interrupt flag which will be set in ISR. */

            for (int i = 0; i < FifoCnt; i++)
            {
                SampleMean += ADCBuff[i] & 0xffff;
            }
            SampleMean /= FifoCnt;
            Cell_volt = -AD5940_ADCCode2Volt((uint32_t)SampleMean, dsp_cfg.ADCBaseCfg.ADCPga, AppRAMPCfg.ADCRefVolt);

            //calculate voltage from average block output
            //The mean result already removed 32768. So to calculate the voltage, assume mean result is n, use below equation.
            //      Voltage = n/32768*Vref
            //Cell_volt = -AD5940_ADCCode2Volt((ADCBuff[0] & 0xffff) + 32768, AppRAMPCfg.AdcPgaGain, AppRAMPCfg.ADCRefVolt);

#ifdef ADI_DEBUG
            snprintf(floatBuffer, sizeof floatBuffer, "%f", Cell_volt);
            ADI_Print("Vcell = %s mV\n", floatBuffer);
#endif
        }

#ifdef START_EQUI_POT
        //change start voltage of ramp (in mV!)
        AppRAMPCfg.RampStartVolt = *Cell_volt;
#ifdef ADI_DEBUG
        ADI_Print("Changed Vstart to equilibrium potential!\n");
#endif
#endif
    }
}

/**
 * @}
 * @}
*/
