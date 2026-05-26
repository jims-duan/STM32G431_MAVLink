#include "led.h"

LED_Structure led;  // 定义led结构体

/*初始化LED*/
LED_Structure LED_Init(GPIO_TypeDef* gpio, uint16_t pin, LED_POLARITY polarity)
{
    LED_Structure led;

    led.GPIOx = gpio;
    led.GPIO_Pin = pin;
    led.polarity = polarity;

    return led;
}

/*设置LED状态*/
void LED_SetState(LED_Structure* led, LED_STATE state)
{
    GPIO_PinState PinState;

    if(led == NULL) return;
    
    // 判断极性
    if(led->polarity == LED_POLARITY_LOW)
    {
        PinState = (state == LED_STATE_ON) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }
    else
    {
        PinState = (state == LED_STATE_ON) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }
    
    LED_SET_STATE(led->GPIOx, led->GPIO_Pin, PinState);
}

/*LED闪烁*/
void LED_Toggle(LED_Structure* led)
{
    LED_TOGGLE_STATE(led->GPIOx,led->GPIO_Pin);
}

