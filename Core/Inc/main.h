/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32u5xx_hal.h"

#include "hci_tl_interface.h"
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
#define P_CE_Pin GPIO_PIN_13
#define P_CE_GPIO_Port GPIOC
#define P_TE_Pin GPIO_PIN_14
#define P_TE_GPIO_Port GPIOC
#define Unused1_Pin GPIO_PIN_15
#define Unused1_GPIO_Port GPIOC
#define Unused2_Pin GPIO_PIN_10
#define Unused2_GPIO_Port GPIOB
#define Unused3_Pin GPIO_PIN_12
#define Unused3_GPIO_Port GPIOB
#define SPI2_CS1_Pin GPIO_PIN_6
#define SPI2_CS1_GPIO_Port GPIOC
#define SPI2_CS2_Pin GPIO_PIN_7
#define SPI2_CS2_GPIO_Port GPIOC
#define SPI2_CS3_Pin GPIO_PIN_8
#define SPI2_CS3_GPIO_Port GPIOC
#define Unused4_Pin GPIO_PIN_9
#define Unused4_GPIO_Port GPIOC
#define C1_INTN_Pin GPIO_PIN_8
#define C1_INTN_GPIO_Port GPIOA
#define C1_INTN_EXTI_IRQn EXTI8_IRQn
#define C2_INTN_Pin GPIO_PIN_9
#define C2_INTN_GPIO_Port GPIOA
#define C2_INTN_EXTI_IRQn EXTI9_IRQn
#define C3_INTN_Pin GPIO_PIN_10
#define C3_INTN_GPIO_Port GPIOA
#define C3_INTN_EXTI_IRQn EXTI10_IRQn
#define CAN_STB_Pin GPIO_PIN_15
#define CAN_STB_GPIO_Port GPIOA
#define Unused5_Pin GPIO_PIN_10
#define Unused5_GPIO_Port GPIOC
#define LED_Ind_Pin GPIO_PIN_11
#define LED_Ind_GPIO_Port GPIOC
#define Unused6_Pin GPIO_PIN_12
#define Unused6_GPIO_Port GPIOC
#define BLE_CS_Pin GPIO_PIN_2
#define BLE_CS_GPIO_Port GPIOD
#define BLE_SPI_SCK_Pin GPIO_PIN_3
#define BLE_SPI_SCK_GPIO_Port GPIOB
#define BLE_SPI_MISO_Pin GPIO_PIN_4
#define BLE_SPI_MISO_GPIO_Port GPIOB
#define BLE_SPI_MOSI_Pin GPIO_PIN_5
#define BLE_SPI_MOSI_GPIO_Port GPIOB
#define BLE_EXTI_Pin GPIO_PIN_6
#define BLE_EXTI_GPIO_Port GPIOB
#define BLE_EXTI_EXTI_IRQn EXTI6_IRQn
#define BLE_RST_Pin GPIO_PIN_7
#define BLE_RST_GPIO_Port GPIOB
#define P_SEL_Pin GPIO_PIN_8
#define P_SEL_GPIO_Port GPIOB
#define P_PROG2_Pin GPIO_PIN_9
#define P_PROG2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
