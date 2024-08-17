#include "USART.h"
#include "n32l40x.h"
#include "n32l40x_usart.h"
#include <stdint.h>

#define countof(a) (sizeof(a) / sizeof(*(a)))
#define TxBufferSize1 (countof(TxBuffer1) - 1)
#define TxBufferSize2 (countof(TxBuffer2) - 1)

USART_InitType USART_InitStructure;
__IO uint8_t USARTy_Tx_Done = 0, USARTy_Rx_Done = 0;
__IO uint8_t USARTz_Tx_Done = 0, USARTz_Rx_Done = 0;
volatile Status TransferStatus1 = FAILED, TransferStatus2 = FAILED;

/**
 * @brief  Configures the nested vectored interrupt controller.
 */
void USART_NVIC_Configuration(void)
{
    NVIC_InitType NVIC_InitStructure;
		/*配置中断*/
		NVIC_InitStructure.NVIC_IRQChannel            = USARTy_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd         = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

    /* Enable the USARTy_Tx DMA Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel            = USARTy_Tx_DMA_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd         = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable the USARTy_Rx DMA Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel            = USARTy_Rx_DMA_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    = 2;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_Init(&NVIC_InitStructure);

}

/**
 * @brief  Configures the different GPIO ports.
 */
void USART_GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);
    
    /* Configure USARTy Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = USARTy_TxPin;    
    GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = USARTy_Tx_GPIO_AF;
    GPIO_InitPeripheral(USARTy_GPIO, &GPIO_InitStructure);

    /* Configure USARTy Rx as alternate function push-pull */
    GPIO_InitStructure.Pin            = USARTy_RxPin;
		GPIO_InitStructure.GPIO_Pull      = GPIO_Pull_Up; //////////////////////
    GPIO_InitStructure.GPIO_Alternate = USARTy_Rx_GPIO_AF;
    GPIO_InitPeripheral(USARTy_GPIO, &GPIO_InitStructure);    
}

/**
 * @brief  Configures the different system clocks.
 */
void USART_RCC_Configuration(void)
{
    /* DMA clock enable */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_DMA, ENABLE);
    /* Enable GPIO clock */
    RCC_EnableAPB2PeriphClk(USARTy_GPIO_CLK | USARTz_GPIO_CLK, ENABLE);
    /* Enable USARTy and USARTz Clock */
    USARTy_APBxClkCmd(USARTy_CLK, ENABLE);
    // USARTz_APBxClkCmd(USARTz_CLK, ENABLE);
}


/**
 * @brief  Configures the DMA.
 */
DMA_InitType DMA_InitStructure;
void DMA_Configuration(uint8_t *data_tx, uint8_t *data_rx)
{
    /* USARTy_Tx_DMA_Channel (triggered by USARTy Tx event) Config */
    DMA_DeInit(USARTy_Tx_DMA_Channel);
    DMA_InitStructure.PeriphAddr     = USARTy_DAT_Base;
    DMA_InitStructure.MemAddr        = (uint32_t)data_tx;
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_DST;
    DMA_InitStructure.BufSize        = USARTy_Tx_Len_INIT;
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_DISABLE;
    DMA_InitStructure.DMA_MemoryInc  = DMA_MEM_INC_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
    DMA_InitStructure.MemDataSize    = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.CircularMode   = DMA_MODE_NORMAL;
    DMA_InitStructure.Priority       = DMA_PRIORITY_VERY_HIGH;
    DMA_InitStructure.Mem2Mem        = DMA_M2M_DISABLE;
    DMA_Init(USARTy_Tx_DMA_Channel, &DMA_InitStructure);
    DMA_RequestRemap(USARTy_Tx_DMA_REMAP, DMA, USARTy_Tx_DMA_Channel, ENABLE);

    /* Enable USARTy Tx DMA Channel Transfer Complete Interrupt */
    DMA_ConfigInt(USARTy_Tx_DMA_Channel, DMA_INT_TXC, ENABLE);

    /* USARTy RX DMA Channel (triggered by USARTy Rx event) Config */
    DMA_DeInit(USARTy_Rx_DMA_Channel);
    DMA_InitStructure.PeriphAddr = USARTy_DAT_Base;
    DMA_InitStructure.MemAddr    = (uint32_t)data_rx;
    DMA_InitStructure.Direction  = DMA_DIR_PERIPH_SRC;
    DMA_InitStructure.BufSize    = USARTy_Rx_Len_IDLE;
    DMA_Init(USARTy_Rx_DMA_Channel, &DMA_InitStructure);
    DMA_RequestRemap(USARTy_Rx_DMA_REMAP, DMA, USARTy_Rx_DMA_Channel, ENABLE);

    /* Enable USARTy Rx DMA Channel Transfer Complete Interrupt */
    DMA_ConfigInt(USARTy_Rx_DMA_Channel, DMA_INT_TXC, ENABLE);
		
		/* Disnable USARTy TX DMA1 Channel */
    DMA_EnableChannel(USARTy_Tx_DMA_Channel, DISABLE);
    /* Enable USARTy RX DMA1 Channel */
    DMA_EnableChannel(USARTy_Rx_DMA_Channel, ENABLE);

    DMA_InitStructure.PeriphAddr     = USARTy_DAT_Base;
    DMA_InitStructure.MemAddr        = (uint32_t)data_tx;
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_DST;
    DMA_InitStructure.BufSize        = USARTy_Tx_Len_INIT;
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_DISABLE;
    DMA_InitStructure.DMA_MemoryInc  = DMA_MEM_INC_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
    DMA_InitStructure.MemDataSize    = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.CircularMode   = DMA_MODE_NORMAL;
    DMA_InitStructure.Priority       = DMA_PRIORITY_VERY_HIGH;
    DMA_InitStructure.Mem2Mem        = DMA_M2M_DISABLE;
}

void dmaSend(uint16_t len) {
    DMA_EnableChannel(USARTy_Tx_DMA_Channel, DISABLE);
    DMA_SetCurrDataCounter(USARTy_Tx_DMA_Channel, len);
    DMA_EnableChannel(USARTy_Tx_DMA_Channel, ENABLE);
}

void usartInit(uint32_t baudrate, uint8_t *data_tx, uint8_t *data_rx) {
    /* System Clocks Configuration */
    USART_RCC_Configuration();

    /* NVIC configuration */
    USART_NVIC_Configuration();

    /* Configure the GPIO ports */
    USART_GPIO_Configuration();

    /* Configure the DMA */
    DMA_Configuration(data_tx, data_rx);

    /* USARTy and USARTz configuration ------------------------------------------------------*/
    USART_InitStructure.BaudRate            = baudrate;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;
    USART_Init(USARTy, &USART_InitStructure);

    /* Enable USARTy DMA Rx and TX request */
    USART_EnableDMA(USARTy, USART_DMAREQ_RX | USART_DMAREQ_TX, ENABLE);
    
    USART_ConfigInt(USARTy, USART_INT_IDLEF, ENABLE);
		
    /* Enable the USARTy and USARTz */
    USART_Enable(USARTy, ENABLE);
}

/**
 * @brief  Compares two buffers.
 * @param  pBuffer1, pBuffer2: buffers to be compared.
 * @param BufferLength buffer's length
 * @return PASSED: pBuffer1 identical to pBuffer2
 *         FAILED: pBuffer1 differs from pBuffer2
 */
Status Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
    while (BufferLength--)
    {
        if (*pBuffer1 != *pBuffer2)
        {
            return FAILED;
        }

        pBuffer1++;
        pBuffer2++;
    }

    return PASSED;
}

