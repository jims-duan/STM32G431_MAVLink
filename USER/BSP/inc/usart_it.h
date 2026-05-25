#ifndef USART_IT_H
#define USART_IT_H

#include "main.h"
#include "usart.h"

#define USART_IT_DEBUG 1

#define USART_RxBUFF_SIZE  1024
typedef struct
{
    volatile uint8_t TxCompleteFlag;         // 发送完成标志位
    uint8_t readBuffer[USART_RxBUFF_SIZE];    
} USART_Structure;
extern USART_Structure LPUSART1_Struct;
extern USART_Structure USART1_Struct;
extern USART_Structure USART3_Struct;


#endif