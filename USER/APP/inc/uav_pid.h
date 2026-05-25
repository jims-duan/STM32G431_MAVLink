#ifndef _UAV_PID_H
#define _UAV_PID_H

#include "main.h"
#include "pid.h"


typedef struct
{
    float PositionOutput[3]; // 位置PID输出变量
    float SpeedOutput[3];    // 速度PID输出变量

    float PositionFeedback[3]; // 位置当前反馈值
    float SpeedFeedback[3];    // 速度当前反馈值

    PID_TypeDef PositionPID[3]; // X位置PID控制器实例
    PID_TypeDef SpeedPID[3];    // X速度PID控制器实例

    float YawOutput;            // yaw使用简单控制即可
} UAV_PID_HandleTypeDef;
extern UAV_PID_HandleTypeDef huav_pid;

void uav_pid_init(void);  /* 初始化PID控制器参数 */
void uav_pid_update(void); /* 定时器更新，更新PID控制器 */

#endif

