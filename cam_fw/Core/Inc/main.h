/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_CAM_Pin GPIO_PIN_13
#define LED_CAM_GPIO_Port GPIOC
#define CAM_HREF_Pin GPIO_PIN_1
#define CAM_HREF_GPIO_Port GPIOB
#define CAM_VSYNC_Pin GPIO_PIN_2
#define CAM_VSYNC_GPIO_Port GPIOB
#define SEN_INT_Pin GPIO_PIN_12
#define SEN_INT_GPIO_Port GPIOB
#define SEN_CS_Pin GPIO_PIN_8
#define SEN_CS_GPIO_Port GPIOA
#define CAM_RST_Pin GPIO_PIN_6
#define CAM_RST_GPIO_Port GPIOC
#define SRAM_CS_Pin GPIO_PIN_7
#define SRAM_CS_GPIO_Port GPIOC
#define CD_CS_Pin GPIO_PIN_15
#define CD_CS_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_1
#define LED_G_GPIO_Port GPIOD
#define LED_R_Pin GPIO_PIN_2
#define LED_R_GPIO_Port GPIOD
#define CD_INT_Pin GPIO_PIN_3
#define CD_INT_GPIO_Port GPIOD
#define CD_RST_Pin GPIO_PIN_9
#define CD_RST_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
