#ifndef _PID_H
#define _PID_H

#include "main.h"


/* PID参数结构体 */
typedef struct
{
    __IO float  SetPoint;            /* 设定目标 */
    __IO float  ActualValue;         /* 期望输出值 */
    __IO float  SumError;            /* 误差累计 */
    __IO float  Proportion;          /* 比例常数 P */
    __IO float  Integral;            /* 积分常数 I */
    __IO float  Derivative;          /* 微分常数 D */
    __IO float  Error;               /* Error[1] */
    __IO float  LastError;           /* Error[-1] */
    __IO float  PrevError;           /* Error[-2] */
} PID_TypeDef;


void pid_init(PID_TypeDef* pid, float kp, float ki, float kd);      /* pid初始化 */
float positional_pid_ctrl(
    PID_TypeDef *pid,
    float Feedback_value, 
    float Integral_limit, 
    float Separation_threshold);  /* pid闭环控制 */

#endif
