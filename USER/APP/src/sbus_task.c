#include "sbus_task.h"
#include "ring_buffer.h"
#include "usbd_cdc_if.h"
#include "uav_pid.h"

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

            if (sbus_frame.channels[7] > 1500)  // A通道打开
            {
                sbus_rc_struct.event = SBUS_RC_EVENT_A_CH8;
            }
            else
            {
                sbus_rc_struct.event = SBUS_RC_EVENT_NOT;
            }
            // 读取各通道值（范围约172-1811）
            switch(sbus_rc_struct.event)
            {
                case(SBUS_RC_EVENT_A_CH8):  // A通道打开
                {
                    // sbus_encode_frame_t.channels[0]  = sbus_frame.channels[0];
                    // sbus_encode_frame_t.channels[1]  = sbus_frame.channels[1];
                    // sbus_encode_frame_t.channels[2]  = sbus_frame.channels[2];

                    sbus_encode_frame_t.channels[0] = sbus_frame.channels[0] + huav_pid.SpeedOutput[0];
                    sbus_encode_frame_t.channels[1] = sbus_frame.channels[1] - huav_pid.SpeedOutput[1];
                    sbus_encode_frame_t.channels[2] = sbus_frame.channels[2] + huav_pid.SpeedOutput[2];
                    
                    sbus_encode_frame_t.channels[3] = sbus_frame.channels[3] - huav_pid.YawOutput;
                }
                break;

                default:
                {
                    sbus_encode_frame_t.channels[0]  = sbus_frame.channels[0];  // 横滚/副翼
                    sbus_encode_frame_t.channels[1]  = sbus_frame.channels[1];  // 俯仰/升降
                    sbus_encode_frame_t.channels[2]  = sbus_frame.channels[2];  // 油门
                    sbus_encode_frame_t.channels[3]  = sbus_frame.channels[3];  // 偏航/方向舵
                }
                break;
            }
            sbus_encode_frame_t.channels[4]  = sbus_frame.channels[4];  // 辅助通道1
            sbus_encode_frame_t.channels[5]  = sbus_frame.channels[5];  // 辅助通道2
            sbus_encode_frame_t.channels[6]  = sbus_frame.channels[6];  // 辅助通道3
            sbus_encode_frame_t.channels[7]  = sbus_frame.channels[7];  // 辅助通道4
            sbus_encode_frame_t.channels[8]  = sbus_frame.channels[8];  // 辅助通道5
            sbus_encode_frame_t.channels[9]  = sbus_frame.channels[9];  // 辅助通道6
            sbus_encode_frame_t.channels[10] = sbus_frame.channels[10]; // 辅助通道7
            sbus_encode_frame_t.channels[11] = sbus_frame.channels[11]; // 辅助通道8
            sbus_encode_frame_t.channels[12] = sbus_frame.channels[12]; // 辅助通道9
            sbus_encode_frame_t.channels[13] = sbus_frame.channels[13]; // 辅助通道10
            sbus_encode_frame_t.channels[14] = sbus_frame.channels[14]; // 辅助通道11
            sbus_encode_frame_t.channels[15] = sbus_frame.channels[15]; // 辅助通道12

            // 合成SBUS帧
            sbus_encode_frame(&sbus_encode_frame_t,encode_frame);
        }
    }
}

// 发送SBUS信号
void SendSBUS()
{
    HAL_UART_Transmit_DMA(&huart3, encode_frame, 25);
}


