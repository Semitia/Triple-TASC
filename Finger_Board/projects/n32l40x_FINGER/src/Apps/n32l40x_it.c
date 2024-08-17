/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file n32g030_it.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "n32l40x_it.h"

#include "USART.h"
#include "mainTask.h"
#include "n32l40x_dma.h"
#include "n32l40x_usart.h"
#include "recvCmdTask.h"

extern __IO uint8_t USARTy_Tx_Done, USARTy_Rx_Done;
extern __IO uint8_t USARTz_Tx_Done, USARTz_Rx_Done;

/** @addtogroup N32G030_StdPeriph_Template
 * @{
 */

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
 * @brief  This function handles NMI exception.
 */
void NMI_Handler(void) {}

/**
 * @brief  This function handles Hard Fault exception.
 */
void HardFault_Handler(void) {
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1) {
    }
}

/**
 * @brief  This function handles SVCall exception.
 */
void SVC_Handler(void) {}

/**
 * @brief  This function handles PendSV_Handler exception.
 */
void PendSV_Handler(void) {}

/**
 * @brief  This function handles SysTick Handler.
 */
void SysTick_Handler(void) {}

/******************************************************************************/
/*                 N32G030 Peripherals Interrupt Handlers                     */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_n32g030.s).                                                 */
/******************************************************************************/

/**
 * @brief  This function handles USARTy_Tx global interrupt request.
 */
void USARTy_Tx_DMA_IRQHandler(void) {
    if (DMA_GetIntStatus(USARTy_Tx_DMA_INT, DMA) != RESET) {
        DMA_EnableChannel(USARTy_Tx_DMA_Channel, DISABLE);
        DMA_ClrIntPendingBit(USARTy_Tx_DMA_INT, DMA);
    }
}

/**
 * @brief  This function handles USARTy IDLE global interrupt request.
 */
void USARTy_IRQHandler(void) {
    uint16_t recv_len = 0;
    if (USART_GetIntStatus(USARTy, USART_INT_IDLEF) != RESET) {
        // Clear the IDLE interrupt flag
        USART_ReceiveData(USARTy);

        DMA_EnableChannel(USARTy_Rx_DMA_Channel, DISABLE);

        USART_ClrIntPendingBit(USARTy, USART_INT_IDLEF);

        recv_len = USARTy_Rx_Len_IDLE - DMA_GetCurrDataCounter(USARTy_Rx_DMA_Channel);

        recvComplete(recv_len);

        DMA_SetCurrDataCounter(USARTy_Rx_DMA_Channel, USARTy_Rx_Len_IDLE);

        DMA_EnableChannel(USARTy_Rx_DMA_Channel, ENABLE);
    }
}

/**
 * @brief  This function handles USARTy_Rx DMA global interrupt request.
 */
void USARTy_Rx_DMA_IRQHandler(void) {
    if (DMA_GetIntStatus(USARTy_Rx_DMA_INT, DMA) != RESET) {
        DMA_EnableChannel(USARTy_Rx_DMA_Channel, DISABLE);
        DMA_ClrIntPendingBit(USARTy_Rx_DMA_INT, DMA);
        DMA_SetCurrDataCounter(USARTy_Rx_DMA_Channel, USARTy_Rx_Len_IDLE);
        DMA_EnableChannel(USARTy_Rx_DMA_Channel, ENABLE);
    }
}

/**
 * @brief  This function handles TIM6 global interrupt request.
 */
void TIM6_IRQHandler(void) {
    if (TIM_GetIntStatus(TIM6, TIM_INT_UPDATE) != RESET) {
        TIM_ClrIntPendingBit(TIM6, TIM_INT_UPDATE);
        taskLoop();
    }
}
/**
 * @brief  This function handles PPP interrupt request.
 */
/*void PPP_IRQHandler(void)
{
}*/

/**
 * @}
 */
