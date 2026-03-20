#include "bsp_rc.h"
#include "main.h"

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver request
    //ʹ��DMA���ڽ���
    SET_BIT(huart5.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
    //ʹ�ܿ����ж�
    do{
    
    __HAL_DMA_DISABLE(&hdma_uart5_rx);
  }while(hdma_uart5_rx.Instance->CR & DMA_SxCR_EN);//(hdma_uart5_rx.Instance->CR & DMA_SxCR_EN);
   

    //disable DMA
    //ʧЧDMA
    // __HAL_DMA_DISABLE(&hdma_uart5_rx);
    // while(hdma_uart5_rx.Instance->CR & DMA_SxCR_EN)
    // {
    //     __HAL_DMA_DISABLE(&hdma_uart5_rx);
    // }
    

    hdma_uart5_rx.Instance->PAR = (uint32_t) & (UART5->RDR);
    //memory buffer 1
    //�ڴ滺����1
    hdma_uart5_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //�ڴ滺����2
    hdma_uart5_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //���ݳ���
    hdma_uart5_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //ʹ��˫������
    SET_BIT(hdma_uart5_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //ʹ��DMA
    __HAL_DMA_ENABLE(&hdma_uart5_rx);
__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);

}
void RC_unable(void)
{
    __HAL_UART_DISABLE(&huart5);
}

void RC_restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart5);
    __HAL_DMA_DISABLE(&hdma_uart5_rx);

    hdma_uart5_rx.Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_uart5_rx);
    __HAL_UART_ENABLE(&huart5);

}







