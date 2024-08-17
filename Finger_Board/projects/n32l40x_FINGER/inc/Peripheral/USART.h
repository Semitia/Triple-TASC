/**
 * @file USART.h
 * @author Alexavier
 * @brief
 * @version 0.1
 * @date 2024-07-15
 *
 * @copyright Copyright (c) 2024
 */

#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "n32l40x.h"
#include "user_lib.h"

#define _USART1_USART2_

#ifdef _USART1_USART2_
#define USARTy                   USART1
#define USARTy_CLK               RCC_APB2_PERIPH_USART1
#define USARTy_GPIO              GPIOA
#define USARTy_GPIO_CLK          RCC_APB2_PERIPH_GPIOA
#define USARTy_RxPin             GPIO_PIN_10
#define USARTy_TxPin             GPIO_PIN_9
#define USARTy_Rx_GPIO_AF        GPIO_AF4_USART1
#define USARTy_Tx_GPIO_AF        GPIO_AF4_USART1
#define USARTy_APBxClkCmd        RCC_EnableAPB2PeriphClk
#define USARTy_DAT_Base          (USART1_BASE + 0x04)
#define USARTy_Tx_DMA_Channel    DMA_CH4
#define USARTy_Tx_DMA_FLAG       DMA_FLAG_TC4
#define USARTy_Tx_DMA_IRQn       DMA_Channel4_IRQn
#define USARTy_Tx_DMA_IRQHandler DMA_Channel4_IRQHandler
#define USARTy_Tx_DMA_INT        DMA_INT_TXC4
#define USARTy_Rx_DMA_Channel    DMA_CH5
#define USARTy_Rx_DMA_FLAG       DMA_FLAG_TC5
#define USARTy_Rx_DMA_IRQn       DMA_Channel5_IRQn
#define USARTy_Rx_DMA_IRQHandler DMA_Channel5_IRQHandler
#define USARTy_Rx_DMA_INT        DMA_INT_TXC5
#define USARTy_Tx_DMA_REMAP      DMA_REMAP_USART1_TX
#define USARTy_Rx_DMA_REMAP      DMA_REMAP_USART1_RX
#define USARTy_Rx_Len_IDLE       (100)
#define USARTy_Tx_Len_INIT       (100)
#define USARTy_IRQn              USART1_IRQn
#define USARTy_IRQHandler        USART1_IRQHandler

#define USARTz                   USART2
#define USARTz_CLK               RCC_APB1_PERIPH_USART2
#define USARTz_GPIO              GPIOA
#define USARTz_GPIO_CLK          RCC_APB2_PERIPH_GPIOA
#define USARTz_RxPin             GPIO_PIN_3
#define USARTz_TxPin             GPIO_PIN_2
#define USARTz_Rx_GPIO_AF        GPIO_AF4_USART2
#define USARTz_Tx_GPIO_AF        GPIO_AF4_USART2
#define USARTz_APBxClkCmd        RCC_EnableAPB1PeriphClk
#define USARTz_DAT_Base          (USART2_BASE + 0x04)
#define USARTz_DAT_Base          (USART2_BASE + 0x04)
#define USARTz_Tx_DMA_Channel    DMA_CH2
#define USARTz_Tx_DMA_FLAG       DMA_FLAG_TC2
#define USARTz_Tx_DMA_IRQn       DMA_Channel1_2_IRQn
#define USARTz_Tx_DMA_IRQHandler DMA_Channel1_2_IRQHandler
#define USARTz_Tx_DMA_INT        DMA_INT_TXC2
#define USARTz_Rx_DMA_Channel    DMA_CH3
#define USARTz_Rx_DMA_FLAG       DMA_FLAG_TC3
#define USARTz_Rx_DMA_IRQn       DMA_Channel3_4_IRQn
#define USARTz_Rx_DMA_IRQHandler DMA_Channel3_4_IRQHandler
#define USARTz_Rx_DMA_INT        DMA_INT_TXC3
#define USARTz_Tx_DMA_REMAP      DMA_REMAP_USART2_TX
#define USARTz_Rx_DMA_REMAP      DMA_REMAP_USART2_RX
#endif

void dmaSend(uint16_t len);
void usartInit(uint32_t baudrate, uint8_t* data_tx, uint8_t* data_rx);
Status Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */
