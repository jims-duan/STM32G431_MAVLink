#ifndef SBUS_TASK_H
#define SBUS_TASK_H

#include "main.h"
#include "usart.h"
#include "sbus.h"

#define SBUS_TASK_DEBUG 1

extern uint8_t encode_frame[SBUS_FRAME_SIZE];

// 遥控事件
typedef enum
{
    SBUS_RC_EVENT_NOT,             // 无事件
    SBUS_RC_EVENT_A_CH8,           // A按钮打开
    SBUS_RC_EVENT_COUNT
} SBUS_RC_EVENT;

typedef struct
{
    SBUS_RC_EVENT event;
} SBUS_RC_Structure;
extern SBUS_RC_Structure sbus_rc_struct;

void SBUS_ParseTask(void);
void SendSBUS(void);


#endif