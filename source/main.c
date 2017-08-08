/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

///////////////////////////////////////////////////////////////////////////////
//  Includes
///////////////////////////////////////////////////////////////////////////////
// SDK Included Files
#include <string.h>
#include "board.h"
#include "fsl_clock_manager.h"
#include "fsl_lpuart_driver.h"
#include "fsl_lptmr_driver.h"
#include "fsl_debug_console.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
// Timer period: 500000uS
typedef enum    FTE_IOEX_TASK_STATUS_ENUM
{
    FTE_IOEX_STATE_IDLE,
    FTE_IOEX_STATE_RX_INPROGRESS,
    FTE_IOEX_STATE_RX_DONE,
    FTE_IOEX_STATE_PROCESSING,
    FTE_IOEX_STATE_ERROR
}   FTE_IOEX_TASK_STATE, * FTE_IOEX_TASK_STATE_PTR;


///////////////////////////////////////////////////////////////////////////////
//  Consts
///////////////////////////////////////////////////////////////////////////////

#define FTE_IOEX_MS_PER_TICK                    10U   // ms
#define FTE_IOEX_DEFAULT_TRANSMITION_INTERVAL   5000U // ms
#define FTE_IOEX_DEFAULT_RX_FRAME_TIMEOUT       20U
#define FTE_IOEX_MAX_DI                         16
#define FTE_IOEX_DISCRETE_INPUT_START_ADDRESS   0x2000
#define FTE_IOEX_DISCRETE_INPUT_COUNT           FTE_IOEX_MAX_DI
#define FTE_IOEX_HOLDING_REGISTER_START_ADDRESS 0x4000
#define FTE_IOEX_HOLDING_REGISTER_COUNT         2

#define FTE_IOEX_CONFIG_REGISTER_HOLDTIME_ADDRESS   0x6000
#define FTE_IOEX_DI_HOLDTIME                        5000
#define FTE_IOEX_CONFIG_REGISTER_COUNT              1
#define FTE_IOEX_CONFIG_REGISTER_START_ADDRESS      0x6000

///////////////////////////////////////////////////////////////////////////////
//  Variables
///////////////////////////////////////////////////////////////////////////////

uint32_t    ulTicks = 0;
uint16_t    usTransmissionInterval = FTE_IOEX_DEFAULT_TRANSMITION_INTERVAL / FTE_IOEX_MS_PER_TICK;
uint16_t    usRxFrameTimeout = FTE_IOEX_DEFAULT_RX_FRAME_TIMEOUT / FTE_IOEX_MS_PER_TICK;

uint8_t     bDIs[16] = { 0, };
uint32_t    ulHoldTimes[16] = { 0, };
uint32_t    ulHoldTime = FTE_IOEX_DI_HOLDTIME;
uint8_t     pRxBuff[32];
uint8_t     pTxFrame[32];
uint8_t     pRxFrame[32];
lptmr_state_t   lptmrState;
lpuart_state_t  lpuartState;

    
////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////
extern  uint16_t crc16(uint16_t uiCRC, const uint8_t *pData, uint16_t uiLen);
void FTE_IOEX_systemInit(void);


/*!
 * @brief LPTMR interrupt call back function.
 * The function is used to toggle LED1.
 */
void FTE_IOEX_TICK_update(void)
{
    ulTicks++;
}

uint32_t FTE_IOEX_makeReadDiscreteInputs(uint8_t * pFrame, uint32_t ulMaxSize, uint32_t ulStartAddress, uint32_t ulCount)
{
    uint16_t        uiCRC;
    int8_t          nTxFrameLen = 0;

    if ((FTE_IOEX_DISCRETE_INPUT_START_ADDRESS <= ulStartAddress) &&
        (ulStartAddress + ulCount <= FTE_IOEX_DISCRETE_INPUT_START_ADDRESS + FTE_IOEX_DISCRETE_INPUT_COUNT))
    {    
        pFrame[nTxFrameLen++] = 0x00;
        pFrame[nTxFrameLen++] = 0x02;
        pFrame[nTxFrameLen++] = ulCount;
        
        for(int i = 0 ; i < ulCount ; i++)
        {
            pFrame[nTxFrameLen++] = bDIs[(ulStartAddress - FTE_IOEX_DISCRETE_INPUT_START_ADDRESS) + i];
        }
    }
    else 
    {
        pFrame[nTxFrameLen++] = 0x00;
        pFrame[nTxFrameLen++] = 0x82;
        if ((ulCount == 0) || (ulCount > 0x07D0))
        {
            pFrame[nTxFrameLen++] = 0x03;
        }
        else
        {
            pFrame[nTxFrameLen++] = 0x02;
        }
    }
    
    uiCRC = crc16(0xFFFF, pFrame, nTxFrameLen);
    
    pFrame[nTxFrameLen++] = (uiCRC >> 8) & 0xFF;
    pFrame[nTxFrameLen++] = (uiCRC     ) & 0xFF;
    
    return  nTxFrameLen;
}

uint32_t FTE_IOEX_makeReadHoldingRegisters(uint8_t * pFrame, uint32_t ulMaxSize, uint32_t ulStartAddress, uint32_t ulCount)
{
    uint16_t        uiCRC;
    int8_t          nTxFrameLen = 0;

    if ((FTE_IOEX_HOLDING_REGISTER_START_ADDRESS <= ulStartAddress) &&
        (ulStartAddress + ulCount <= FTE_IOEX_HOLDING_REGISTER_START_ADDRESS + FTE_IOEX_HOLDING_REGISTER_COUNT))
    {    
        pFrame[nTxFrameLen++] = 0x00;
        pFrame[nTxFrameLen++] = 0x02;
        pFrame[nTxFrameLen++] = ulCount;
        
        for(int i = 0 ; i < ulCount ; i++)
        {
            uint16_t usOffset = ulStartAddress - FTE_IOEX_HOLDING_REGISTER_START_ADDRESS + i;

            switch(usOffset)
            {
            case    0:
                {
                    pFrame[nTxFrameLen++] = (usTransmissionInterval >> 8) & 0xFF;
                    pFrame[nTxFrameLen++] = (usTransmissionInterval     ) & 0xFF;
                }
                break;
                
            case    1:
                {
                    pFrame[nTxFrameLen++] = (usRxFrameTimeout >> 8) & 0xFF;
                    pFrame[nTxFrameLen++] = (usRxFrameTimeout     ) & 0xFF;
                }
                break;
            }   
        }
    }
    else 
    {
        pFrame[nTxFrameLen++] = 0x00;
        pFrame[nTxFrameLen++] = 0x82;
        if ((ulCount == 0) || (ulCount > 0x07D0))
        {
            pFrame[nTxFrameLen++] = 0x03;
        }
        else
        {
            pFrame[nTxFrameLen++] = 0x02;
        }
    }
    
    uiCRC = crc16(0xFFFF, pFrame, nTxFrameLen);
    
    pFrame[nTxFrameLen++] = (uiCRC >> 8) & 0xFF;
    pFrame[nTxFrameLen++] = (uiCRC     ) & 0xFF;
    
    return  nTxFrameLen;
}

uint32_t FTE_IOEX_writeSingleRegister(uint8_t * pFrame, uint32_t ulMaxSize, uint16_t usAddress, uint16_t usValue)
{
    uint16_t        uiCRC;
    int8_t          nTxFrameLen = 0;

    if ((FTE_IOEX_HOLDING_REGISTER_START_ADDRESS <= usAddress) &&
        (usAddress < FTE_IOEX_HOLDING_REGISTER_START_ADDRESS + FTE_IOEX_HOLDING_REGISTER_COUNT))
    {    
        pFrame[nTxFrameLen++] = 0x00;
        pFrame[nTxFrameLen++] = 0x02;
        pFrame[nTxFrameLen++] = (usAddress >> 8) & 0xFF;
        pFrame[nTxFrameLen++] = (usAddress     ) & 0xFF;
        pFrame[nTxFrameLen++] = (usValue >> 8) & 0xFF;
        pFrame[nTxFrameLen++] = (usValue     ) & 0xFF;
        
        switch(usAddress - FTE_IOEX_HOLDING_REGISTER_START_ADDRESS)
        {
        case    0:
            {
                usTransmissionInterval = usValue;
            }
            break;
            
        case    1:
            {
                usRxFrameTimeout  = usValue;
            }
            break;
        }   
    }
    else if ((FTE_IOEX_CONFIG_REGISTER_START_ADDRESS <= usAddress) &&
        (usAddress < FTE_IOEX_CONFIG_REGISTER_START_ADDRESS + FTE_IOEX_CONFIG_REGISTER_COUNT))
    {    
        pFrame[nTxFrameLen++] = 0x00;
        pFrame[nTxFrameLen++] = 0x02;
        pFrame[nTxFrameLen++] = (usAddress >> 8) & 0xFF;
        pFrame[nTxFrameLen++] = (usAddress     ) & 0xFF;
        pFrame[nTxFrameLen++] = (usValue >> 8) & 0xFF;
        pFrame[nTxFrameLen++] = (usValue     ) & 0xFF;
        
        switch(usAddress)
        {
        case    FTE_IOEX_CONFIG_REGISTER_HOLDTIME_ADDRESS:
            {
                ulHoldTime = usValue / 10;
            }
            break;
        }   
    }
    else 
    {
        pFrame[nTxFrameLen++] = 0x00;
        pFrame[nTxFrameLen++] = 0x82;
        pFrame[nTxFrameLen++] = 0x02;
    }
    
    uiCRC = crc16(0xFFFF, pFrame, nTxFrameLen);
    
    pFrame[nTxFrameLen++] = (uiCRC >> 8) & 0xFF;
    pFrame[nTxFrameLen++] = (uiCRC     ) & 0xFF;
    
    return  nTxFrameLen;
}

uint32_t FTE_IOEX_makeFunctionNotSupported(uint8_t * pFrame, uint32_t ulMaxSize, uint8_t ucFunctionCode)
{
    uint16_t    uiCRC;
    int8_t      nTxFrameLen = 0;

    pFrame[nTxFrameLen++] = 0x00;
    pFrame[nTxFrameLen++] = ucFunctionCode + 0x80;
    pFrame[nTxFrameLen++] = 0x01;
    
    uiCRC = crc16(0xFFFF, pFrame, nTxFrameLen);
    
    pFrame[nTxFrameLen++] = (uiCRC >> 8) & 0xFF;
    pFrame[nTxFrameLen++] = (uiCRC     ) & 0xFF;
    
    return  nTxFrameLen;
}

/*!
 * @brief Main function
 */
int main (void)
{
    uint32_t ulTimeout  = usRxFrameTimeout;
    uint32_t ulValidFrameSendTime = 0;
    uint32_t ulPrevRemaining = 0;
    uint32_t ulPrevTicks = 0;
    uint32_t ulRecvLen = 0;
    FTE_IOEX_TASK_STATE xState = FTE_IOEX_STATE_IDLE;

    FTE_IOEX_systemInit();

    ulPrevRemaining = sizeof(pRxBuff);
    LPUART_DRV_ReceiveData(BOARD_DEBUG_UART_INSTANCE, pRxBuff, sizeof(pRxBuff));
    
    while(1)
    {
        int         i;
        uint8_t     bChanged = false;
        uint32_t    ulRemaining = 0;
        uint32_t    nTxFrameLen = 0;
        
        for(i = 0 ; i < FTE_IOEX_MAX_DI ; i++)
        {
            if (rand() % 2)
//            if (bDIs[i] != GPIO_DRV_ReadPinInput(inputPins[i].pinName))
            {
                if ((ulHoldTimes[i] == 0) || ((ulTicks - ulHoldTimes[i]) >= 500))
                {
                    bDIs[i] = !bDIs[i];
                    ulHoldTimes[i] = ulTicks;
                    bChanged = true;
                }
            }
        }
        
        switch(xState)
        {
        case    FTE_IOEX_STATE_IDLE:
            {
                LPUART_DRV_GetReceiveStatus(BOARD_DEBUG_UART_INSTANCE, &ulRemaining);
                if (ulPrevRemaining != ulRemaining)
                {
                    ulPrevTicks = ulTicks;
                    ulPrevRemaining = ulRemaining;
                    xState = FTE_IOEX_STATE_RX_INPROGRESS;
                }

                if (bChanged || ((ulTicks - ulValidFrameSendTime) >= usTransmissionInterval))
                {            
                    nTxFrameLen = FTE_IOEX_makeReadDiscreteInputs(pTxFrame, sizeof(pTxFrame), FTE_IOEX_DISCRETE_INPUT_START_ADDRESS, FTE_IOEX_DISCRETE_INPUT_COUNT);
                    ulValidFrameSendTime = ulTicks;            
                }        
            }
            break;
            
        case    FTE_IOEX_STATE_RX_INPROGRESS:
            {
                LPUART_DRV_GetReceiveStatus(BOARD_DEBUG_UART_INSTANCE, &ulRemaining);
                if (ulPrevRemaining != ulRemaining)
                {
                    ulPrevTicks = ulTicks;
                    ulPrevRemaining = ulRemaining;
                }
                else if ((ulTicks - ulPrevTicks) >= ulTimeout)
                {
                    ulTimeout = usRxFrameTimeout;
                    xState = FTE_IOEX_STATE_RX_DONE;
                }
            }
            break;
            
        case    FTE_IOEX_STATE_RX_DONE:
            {
                LPUART_DRV_GetReceiveStatus(BOARD_DEBUG_UART_INSTANCE, &ulRemaining);
                
                LPUART_DRV_AbortReceivingData(BOARD_DEBUG_UART_INSTANCE);
                ulRecvLen = sizeof(pRxBuff) - ulRemaining;
                if (ulRecvLen > 4)
                {
                    memcpy(pRxFrame, pRxBuff, ulRecvLen);    
                    xState = FTE_IOEX_STATE_PROCESSING;
                }
                else
                {
                    ulRecvLen = 0;
                    xState = FTE_IOEX_STATE_IDLE;
                }
                    
                memset(pRxBuff, 0, sizeof(pRxBuff));
                ulPrevRemaining = sizeof(pRxBuff);
                LPUART_DRV_ReceiveData(BOARD_DEBUG_UART_INSTANCE, pRxBuff, sizeof(pRxBuff));
                
                ulTimeout = usRxFrameTimeout;
            }
            break;
            
        case    FTE_IOEX_STATE_PROCESSING:
            {
                uint16_t uiCRC = crc16(0xFFFF, (uint8_t *)pRxFrame, ulRecvLen - 2);                 
                if ( (((uint16_t)pRxFrame[ulRecvLen - 2] << 8) | pRxFrame[ulRecvLen - 1]) == uiCRC)
                {
                    switch(pRxFrame[1])
                    {
                    case    0x02:
                        {
                            uint16_t    usAddress  = ((uint16_t)pRxFrame[2] << 8) | pRxFrame[3];
                            uint16_t    usCount    = ((uint16_t)pRxFrame[4] << 8) | pRxFrame[5];
                            
                            nTxFrameLen = FTE_IOEX_makeReadDiscreteInputs(pTxFrame, sizeof(pTxFrame), usAddress, usCount);
                            ulValidFrameSendTime = ulTicks;            
                        }
                        break;
                        
                    case    0x03:
                        {
                            uint16_t    usAddress  = ((uint16_t)pRxFrame[2] << 8) | pRxFrame[3];
                            uint16_t    usCount    = ((uint16_t)pRxFrame[4] << 8) | pRxFrame[5];
                            
                            nTxFrameLen = FTE_IOEX_makeReadHoldingRegisters(pTxFrame, sizeof(pTxFrame), usAddress, usCount);
                            ulValidFrameSendTime = ulTicks;            
                        }
                        break;
                        
                    case    0x06:
                        {
                            uint16_t    usAddress  = ((uint16_t)pRxFrame[2] << 8) | pRxFrame[3];
                            uint16_t    usValue    = ((uint16_t)pRxFrame[4] << 8) | pRxFrame[5];
                            
                            nTxFrameLen = FTE_IOEX_writeSingleRegister(pTxFrame, sizeof(pTxFrame), usAddress, usValue);
                            ulValidFrameSendTime = ulTicks;            
                        }
                        break;
                        
                    default:
                        {
                            nTxFrameLen = FTE_IOEX_makeFunctionNotSupported(pTxFrame, sizeof(pTxFrame), pRxFrame[1]);
                        }
                    }    
                }
                ulRecvLen = 0;
                xState = FTE_IOEX_STATE_IDLE;

           }
            break;
            
        case    FTE_IOEX_STATE_ERROR:
            {
            }
            break;
        }
        
        if (nTxFrameLen != 0)
        {
            LPUART_DRV_SendData(BOARD_DEBUG_UART_INSTANCE, pTxFrame, nTxFrameLen);
            nTxFrameLen = 0;
        }
    }
}


void FTE_IOEX_systemInit(void)
{
    // LPTMR configurations
    lptmr_user_config_t lptmrConfig =
    {
        .timerMode = kLptmrTimerModeTimeCounter,
        .freeRunningEnable = false,
        .prescalerEnable = true,
        .prescalerClockSource = kClockLptmrSrcLpoClk,
        .prescalerValue = kLptmrPrescalerDivide2,
        .isInterruptEnabled = true,
    };
    // LPTMR driver state information
    
    // Initialize standard SDK demo application pins
    hardware_init();

    GPIO_DRV_Init(inputPins, outputPins);
    
    // Initialize LPTMR
    LPTMR_DRV_Init(LPTMR0_IDX, &lptmrState, &lptmrConfig);
    // Set timer period for TMR_PERIOD seconds
    LPTMR_DRV_SetTimerPeriodUs(LPTMR0_IDX, FTE_IOEX_MS_PER_TICK * 1000);
    // Install interrupt call back function for LPTMR
    LPTMR_DRV_InstallCallback(LPTMR0_IDX, FTE_IOEX_TICK_update);
    // Start LPTMR
    LPTMR_DRV_Start(LPTMR0_IDX);

    uint8_t rxChar = 0, txChar = 0;
    uint32_t byteCountBuff = 0;

    // Fill in lpuart config data
    lpuart_user_config_t lpuartConfig = {
        .clockSource     = BOARD_LPUART_CLOCK_SOURCE,
        .bitCountPerChar = kLpuart8BitsPerChar,
        .parityMode      = kLpuartParityDisabled,
        .stopBitCount    = kLpuartOneStopBit,
        .baudRate        = BOARD_DEBUG_UART_BAUD
    };

    // Initialize the lpuart module with instance number and config structure
    LPUART_DRV_Init(BOARD_DEBUG_UART_INSTANCE, &lpuartState, &lpuartConfig);
}