#include "pid.h"


/**
 * @brief       pid初始化
 * @param       无
 * @retval      无
 */
void pid_init(PID_TypeDef* pid, float kp, float ki, float kd)
{
    pid->SetPoint = 0;       /* 设定目标值 */
    pid->ActualValue = 0.0;  /* 期望输出值 */
    pid->SumError = 0.0;     /* 积分值 */
    pid->Error = 0.0;        /* Error[1] */
    pid->LastError = 0.0;    /* Error[-1] */
    pid->PrevError = 0.0;    /* Error[-2] */
    pid->Proportion = kp;    /* 比例常数 Proportional Const */
    pid->Integral = ki;      /* 积分常数 Integral Const */
    pid->Derivative = kd;    /* 微分常数 Derivative Const */ 
}

/**
 * @brief       pid闭环控制
 * @param       *pid：pid结构体变量地址
 * @param       Feedback_value：当前实际值
 * @retval      期望输出值
 */
float positional_pid_ctrl(
    PID_TypeDef *pid,
    float Feedback_value, 
    float Integral_limit, 
    float Separation_threshold)
{
    pid->Error = (float)(pid->SetPoint - Feedback_value);                   /* 计算偏差 */

    // 分离式PID：当误差较小时，引入积分项；当误差较大时，暂不计算积分项，防止积分过大导致控制信号过大
    if (fabs(pid->Error) < Separation_threshold)
    {
        pid->SumError += pid->Error;
        // 积分限幅，防止积分过大导致控制信号过大
        if(pid->SumError >= Integral_limit) pid->SumError = Integral_limit;
        if(pid->SumError <= -Integral_limit) pid->SumError = -Integral_limit;
    }
    else
    {
        pid->SumError = 0.0f; // 误差较大时，重置积分项
    }

    pid->ActualValue = (pid->Proportion * pid->Error)                       /* 比例环节 */
                       + (pid->Integral * pid->SumError)                    /* 积分环节 */
                       + (pid->Derivative * (pid->Error - pid->LastError)); /* 微分环节 */
    pid->LastError = pid->Error;
    
    return ((float)(pid->ActualValue));                                   /* 返回计算后输出的数值 */
}
