#include "tim_it.h"
#include "pid.h"
#include "uav_pid.h"
#include "sbus_task.h"

// 定时器中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // 5ms定时器，用于计算PID
  if (htim->Instance == TIM6)
  {
    // 更新PID控制器
    uav_pid_update();
  }
  // 14ms定时器，RC信号输出
  else if (htim->Instance == TIM16)
  {
    SendSBUS();
  }
}

