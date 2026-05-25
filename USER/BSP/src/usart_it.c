#include "usart_it.h"
#include "ring_buffer.h"
#include "led_fsm.h"
#include "usmart.h"

USART_Structure LPUSART1_Struct = 
{
  .TxCompleteFlag = 1,
};
USART_Structure USART1_Struct = 
{
  .TxCompleteFlag = 1,
};
USART_Structure USART3_Struct = 
{
  .TxCompleteFlag = 1,
};

// 串口接收事件回调函数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &hlpuart1)
  {
    // 将接收到的数据复制到usmart缓冲区
    memcpy(usmart_buf,LPUSART1_Struct.readBuffer,Size);
    HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, LPUSART1_Struct.readBuffer, USART_RxBUFF_SIZE);
  }
  else if (huart == &huart3)
  {
    // 写入缓冲区
    RingBuff_WriteBytes(&u3RxRingBufferStruct,USART3_Struct.readBuffer,Size);
    // 重启DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, USART3_Struct.readBuffer, USART_RxBUFF_SIZE);
  }
}

// 串口发送完成回调
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    USART1_Struct.TxCompleteFlag = 1;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &hlpuart1)
  {
    // 获取错误类型
    uint32_t error_code = huart->ErrorCode;
    
    // 停止DMA传输
    HAL_UART_DMAStop(huart);
    
    // 清除所有错误标志位
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_PE);
    
    // 重新初始化串口（可选，根据错误严重程度）
    if (error_code & (HAL_UART_ERROR_FE | HAL_UART_ERROR_NE | HAL_UART_ERROR_PE))
    {
      // 帧错误或噪声错误，需要重新初始化
      HAL_UART_DeInit(huart);
      HAL_UART_Init(huart);
    }
    
    // 重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, LPUSART1_Struct.readBuffer, USART_RxBUFF_SIZE);
  }
  else if (huart == &huart3)
  {
    uint32_t error_code = huart->ErrorCode;
    
    // 停止DMA
    HAL_UART_DMAStop(huart);
    
    // 清除错误标志
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_PE);
    
    // 根据错误类型决定是否需要重新初始化
    if (error_code & (HAL_UART_ERROR_FE | HAL_UART_ERROR_NE))
    {
      HAL_UART_DeInit(huart);
      HAL_UART_Init(huart);
    }
    
    // 重启DMA
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, USART3_Struct.readBuffer, USART_RxBUFF_SIZE);
  }
}
