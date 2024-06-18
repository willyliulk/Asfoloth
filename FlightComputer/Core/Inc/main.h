/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include <IMU_rel/IMU_ALL.h>

#include "stdint.h"
//LoRa related
#include "LoRa_rel/sx127x_io.h"
#include "LoRa_rel/sx127x_config.h"
#include "LoRa_rel/sx127x_reg.h"

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
#define SPI4_CS_Pin GPIO_PIN_4
#define SPI4_CS_GPIO_Port GPIOE
#define DIO2_Pin GPIO_PIN_9
#define DIO2_GPIO_Port GPIOF
#define DIO0_Pin GPIO_PIN_10
#define DIO0_GPIO_Port GPIOF
#define Mode_Switch_Pin GPIO_PIN_6
#define Mode_Switch_GPIO_Port GPIOA
#define IMU_TX_Pin GPIO_PIN_8
#define IMU_TX_GPIO_Port GPIOD
#define IMU_RX_Pin GPIO_PIN_9
#define IMU_RX_GPIO_Port GPIOD
#define FEM_CTX_Pin GPIO_PIN_12
#define FEM_CTX_GPIO_Port GPIOD
#define LoRa_RST_Pin GPIO_PIN_13
#define LoRa_RST_GPIO_Port GPIOD
#define FEM_CPS_Pin GPIO_PIN_14
#define FEM_CPS_GPIO_Port GPIOD
#define DIO1_Pin GPIO_PIN_15
#define DIO1_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

#define EXT_BUFFER_SIZE 512


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
