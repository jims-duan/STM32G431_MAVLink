/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"
#include <string.h>
#include "MavLinkAPP.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : \
                      strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
#define debug_info(fmt, ...) \
    do { \
        mavlink_send_statustext_v(MAV_SEVERITY_INFO, \
            "[INFO] [%s:%d]: " fmt, \
            __FILENAME__, __LINE__, ##__VA_ARGS__); \
    } while(0)

#define debug_warning(fmt, ...) \
    do { \
        mavlink_send_statustext_v(MAV_SEVERITY_WARNING, \
            "[WARN] [%s:%d]: " fmt, \
            __FILENAME__, __LINE__, ##__VA_ARGS__); \
    } while(0)

#define debug_error(fmt, ...) \
    do { \
        mavlink_send_statustext_v(MAV_SEVERITY_ERROR, \
            "[ERROR] [%s:%d]: " fmt, \
            __FILENAME__, __LINE__, ##__VA_ARGS__); \
    } while(0)
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define led_Pin GPIO_PIN_13
#define led_GPIO_Port GPIOC
#define SBUS_Tx_Pin GPIO_PIN_10
#define SBUS_Tx_GPIO_Port GPIOB
#define SBUS_Rx_Pin GPIO_PIN_11
#define SBUS_Rx_GPIO_Port GPIOB
#define GPS_Tx_Pin GPIO_PIN_9
#define GPS_Tx_GPIO_Port GPIOA
#define GPS_Rx_Pin GPIO_PIN_10
#define GPS_Rx_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
