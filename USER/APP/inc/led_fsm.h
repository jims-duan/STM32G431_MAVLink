#ifndef _LED_FSM_H_
#define _LED_FSM_H_

#include "main.h"
#include "led.h"

/*LED FSM状态枚举*/
typedef enum
{
    LED_FSM_STATE_OFF,
    LED_FSM_STATE_ON,
    LED_FSM_STATE_COUNT
} LED_FSM_STATE;

/*LED FSM事件枚举*/
typedef enum
{
    LED_FSM_EVENT_IDLE,
    LED_FSM_EVENT_OFF,
    LED_FSM_EVENT_ON,
    LED_FSM_EVENT_BLINK,
    LED_FSM_EVENT_COUNT
} LED_FSM_EVENT;

/*LED FSM结构体*/
typedef struct
{
    LED_FSM_EVENT event;        // 当前事件
    LED_FSM_STATE state;        // 当前状态
    uint32_t blink_on_tick;     // 闪烁常亮计时
    uint32_t blink_off_tick;    // 闪烁熄灭计时
    uint32_t blink_on_time;     // 闪烁常亮时间(ms)
    uint32_t blink_off_time;    // 闪烁熄灭时间(ms)
    LED_Structure* led_struct;   // led操作对象指针
} LED_FSM_Structure;

/*LED状态机初始化*/
LED_FSM_Structure LED_FSM_Init(LED_Structure* led_struct);
/*LED状态机运行处理函数*/
void LED_FSM_Run(LED_FSM_Structure* fsm, uint32_t tick);
/*设置led闪烁*/
void LED_FSM_SetBlinkEvent(LED_FSM_Structure* fsm, uint32_t on_time, uint32_t off_time);
/*关闭led*/
void LED_FSM_SetOFFEvent(LED_FSM_Structure* fsm);
/*开启led*/
void LED_FSM_SetONEvent(LED_FSM_Structure* fsm);

#endif
