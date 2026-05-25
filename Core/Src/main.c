/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "iwdg.h"
#include "usart.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mavlink.h"
#include "MavLinkAPP.h"
#include "ring_buffer.h"
#include "usart_it.h"
#include "sbus.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include "sbus_task.h"
#include <stdarg.h>
#include "usmart.h"
#include "led.h"
#include "led_fsm.h"
#include "tim_it.h"
#include "pid.h"
#include "odom_gps.h"
#include "uav_pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
LED_Structure led;  // 定义led结构体
LED_FSM_Structure led_fsm; // 定义LED状态机结构体

void set_led_blink(uint32_t on_time, uint32_t off_time)
{
    LED_FSM_SetBlinkEvent(&led_fsm, on_time, off_time);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USB_Device_Init();
  MX_LPUART1_UART_Init();
  MX_TIM6_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  // 启动定时器6(位姿PID控制定时器)
  HAL_TIM_Base_Start_IT(&htim6);
  // 启动定时器16(14ms RC信号输出)
  HAL_TIM_Base_Start_IT(&htim16);
  
  // 初始化MavLink
  MavLink_Init();
  // 启动DMA接收
  HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, LPUSART1_Struct.readBuffer, USART_RxBUFF_SIZE);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, USART3_Struct.readBuffer, USART_RxBUFF_SIZE);
  // 初始化串口接收缓冲区
  RingBuff_Init(&u1TxRingBufferStruct);
  RingBuff_Init(&u3RxRingBufferStruct);

  // 初始化SBUS解码器
  SBUS_Init(&sbus_struct);
  // 设置初始时间戳
  SBUS_SetLastRxTime(&sbus_struct, HAL_GetTick());

  // 初始化LED
  led = LED_Init(led_GPIO_Port,  led_Pin,  LED_POLARITY_LOW);
  // 初始化LED状态机
  led_fsm = LED_FSM_Init(&led);
  // 初始设置500ms闪烁
  LED_FSM_SetBlinkEvent(&led_fsm,500,500);

  // PID参数初始化
  uav_pid_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // 获取系统计数器
    uint32_t tick = HAL_GetTick();

    static uint32_t last_mavlink_time = 0;
    static uint32_t last_iwdg_time = 0;
    
    // 每秒打印一次系统运行时间
    if (tick - last_mavlink_time >= 1100)
    {
      last_mavlink_time = tick;

      // 打印系统运行时间
      // debug_printf("Systemc Running tick: %lu ms\n", tick);
    }

    // 700ms喂狗
    if (tick - last_iwdg_time >= 700)
    {
      last_iwdg_time = tick;
      // 喂狗
      HAL_IWDG_Refresh(&hiwdg);
    }

    // 运行GPS状态机
    Odom_GPS_FSM_Run(tick);

    // 运行LED状态机
    LED_FSM_Run(&led_fsm, tick);

    //执行usmart扫描
    usmart_dev.scan();
    
    // 运行mavlink状态机
    MavLink_FSM(tick);

    // 解码SBUS
    SBUS_ParseTask();
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void _debug_printf(const char *file, int line, const char *format, ...)
{
  uint8_t usbtemp[256];
  uint16_t len;
  
  // 先添加文件名和行号
  int header_len = snprintf((char*)usbtemp, sizeof(usbtemp), "[%s:%d] ", file, line);
  
  va_list args;
  va_start(args, format);
  len = vsnprintf((char*)usbtemp + header_len, sizeof(usbtemp) - header_len, (char*)format, args);
  va_end(args);
  
  len += header_len;
  
  HAL_UART_Transmit(&hlpuart1, usbtemp, len, HAL_MAX_DELAY);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
