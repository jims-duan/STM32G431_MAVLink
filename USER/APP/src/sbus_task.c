#include "sbus_task.h"
#include "ring_buffer.h"
#include "usbd_cdc_if.h"
#include "uav_pid.h"
#include "led_fsm.h"
#include "usart_it.h"

uint8_t encode_frame[SBUS_FRAME_SIZE];

SBUS_RC_Structure sbus_rc_struct = 
{
    .event = SBUS_RC_EVENT_NOT
};

void SBUS_ParseTask(void)
{
    uint8_t byte;
    // 解码SBUS
    while (RingBuff_ReadByte(&u3RxRingBufferStruct, &byte))
    {
        if(SBUS_ParseByte(&sbus_struct, byte, &sbus_frame))
        {
            // 检查帧有效性
            if(!sbus_frame.frame_valid)
            {
#if SBUS_TASK_DEBUG
                debug_printf("SBUS帧无效\r\n");
#endif
                return;
            }
            
            // 检查信号丢失
            if(sbus_frame.frame_lost)
            {
#if SBUS_TASK_DEBUG
                debug_printf("信号丢失\r\n");
#endif
                return;
            }
            
            // 检查故障保护
            if(sbus_frame.failsafe)
            {
#if SBUS_TASK_DEBUG
                debug_printf("故障保护\r\n");
#endif
                return;
            }

            

            if (
                ((sbus_frame.channels[0] > SBUS_NEUTRAL + SBUS_DEADZONE || 
                sbus_frame.channels[0] < SBUS_NEUTRAL - SBUS_DEADZONE) ||     // 通道0: 横滚
                (sbus_frame.channels[1] > SBUS_NEUTRAL + SBUS_DEADZONE || 
                sbus_frame.channels[1] < SBUS_NEUTRAL - SBUS_DEADZONE) ||     // 通道1: 俯仰
                (sbus_frame.channels[2] > SBUS_NEUTRAL + SBUS_DEADZONE || 
                sbus_frame.channels[2] < SBUS_NEUTRAL - SBUS_DEADZONE) ||     // 通道2: 油门
                (sbus_frame.channels[3] > SBUS_NEUTRAL + SBUS_DEADZONE || 
                sbus_frame.channels[3] < SBUS_NEUTRAL - SBUS_DEADZONE))           // 通道3: 偏航
            )
            {
                sbus_rc_struct.event = SBUS_RC_EVENT_MANUAL_CTRL;   // 遥控接管
            }
            else if (
                ((sbus_frame.channels[0] < SBUS_NEUTRAL + SBUS_DEADZONE && 
                sbus_frame.channels[0] > SBUS_NEUTRAL - SBUS_DEADZONE) &&
                (sbus_frame.channels[1] < SBUS_NEUTRAL + SBUS_DEADZONE && 
                sbus_frame.channels[1] > SBUS_NEUTRAL - SBUS_DEADZONE) &&
                (sbus_frame.channels[2] < SBUS_NEUTRAL + SBUS_DEADZONE && 
                sbus_frame.channels[2] > SBUS_NEUTRAL - SBUS_DEADZONE) &&
                (sbus_frame.channels[3] < SBUS_NEUTRAL + SBUS_DEADZONE && 
                sbus_frame.channels[3] > SBUS_NEUTRAL - SBUS_DEADZONE)) && 
                sbus_frame.channels[9] == 1946  // B按钮打开自动控制
            )
            {
                sbus_rc_struct.event = SBUS_RC_EVENT_AUTO_CTRL;  // 自动控制
            }
            else
            {
                sbus_rc_struct.event = SBUS_RC_EVENT_NOT;   // 无事件
            }

            sbus_encode_frame_t = sbus_frame;  // 直接复制原始通道数据，后续根据事件类型进行调整
            // 读取各通道值（范围约172-1811）
            switch(sbus_rc_struct.event)
            {
                case(SBUS_RC_EVENT_MANUAL_CTRL):  // 手动控制
                {
                    // 不进行任何修改，直接使用遥控输入的通道值
                }
                break;

                case(SBUS_RC_EVENT_AUTO_CTRL):  // 自动控制
                {
                    sbus_encode_frame_t.channels[0] = sbus_frame.channels[0] + huav_pid.SpeedOutput[0];
                    sbus_encode_frame_t.channels[1] = sbus_frame.channels[1] - huav_pid.SpeedOutput[1];
                    sbus_encode_frame_t.channels[2] = sbus_frame.channels[2] + huav_pid.SpeedOutput[2];
                    
                    sbus_encode_frame_t.channels[3] = sbus_frame.channels[3] - huav_pid.YawOutput;
                }
                break;

                default:
                {
                    // 不进行任何修改，直接使用遥控输入的通道值
                }
                break;
            }
            // 合成SBUS帧
            sbus_encode_frame(&sbus_encode_frame_t,encode_frame);
        }
    }
}

// 发送SBUS信号
void SendSBUS()
{
    if (USART3_Struct.TxCompleteFlag == 1)
    {
        USART3_Struct.TxCompleteFlag = 0;
        HAL_UART_Transmit_DMA(&huart3, encode_frame, 25);
    }
}


