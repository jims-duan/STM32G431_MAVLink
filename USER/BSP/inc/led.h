#ifndef _LED_H_
#define _LED_H_

#include "main.h"

// GPIO操作接口
#define LED_SET_STATE(port,pin,state)   {\
    HAL_GPIO_WritePin(port,pin,state);  \
}
#define LED_TOGGLE_STATE(port,pin)   {\
    HAL_GPIO_TogglePin(port,pin);  \
}

/*LED极性枚举*/
typedef enum
{
    LED_POLARITY_LOW = 0,   // 低电平点亮（GPIO_RESET = 亮）
    LED_POLARITY_HIGH = 1,  // 高电平点亮（GPIO_SET = 亮）
} LED_POLARITY;

/*LED状态枚举*/
typedef enum
{
    LED_STATE_OFF,
    LED_STATE_ON
} LED_STATE;

/*LED结构体*/
typedef struct
{
    GPIO_TypeDef* GPIOx;        // LED端口
    uint16_t GPIO_Pin;          // LED引脚
    LED_POLARITY polarity;      // LED极性
} LED_Structure;
extern LED_Structure led;  // 定义led结构体

/*初始化LED*/
LED_Structure LED_Init(GPIO_TypeDef* gpio, uint16_t pin, LED_POLARITY polarity);
/*设置LED状态*/
void LED_SetState(LED_Structure* led,LED_STATE state);
/*LED闪烁*/
void LED_Toggle(LED_Structure* led);

#endif
