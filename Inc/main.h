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
#include "stm32g4xx_hal.h"

#include "motorcontrol.h"

#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_cordic.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_gpio.h"

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
#define Start_Stop_Pin LL_GPIO_PIN_13
#define Start_Stop_GPIO_Port GPIOC
#define Start_Stop_EXTI_IRQn EXTI15_10_IRQn
#define M1_CURR_AMPL_V_Pin LL_GPIO_PIN_1
#define M1_CURR_AMPL_V_GPIO_Port GPIOC
#define M1_TEMPERATURE_Pin LL_GPIO_PIN_2
#define M1_TEMPERATURE_GPIO_Port GPIOC
#define M1_CURR_AMPL_U_Pin LL_GPIO_PIN_0
#define M1_CURR_AMPL_U_GPIO_Port GPIOA
#define M1_BUS_VOLTAGE_Pin LL_GPIO_PIN_1
#define M1_BUS_VOLTAGE_GPIO_Port GPIOA
#define UART_TX_Pin LL_GPIO_PIN_2
#define UART_TX_GPIO_Port GPIOA
#define UART_RX_Pin LL_GPIO_PIN_3
#define UART_RX_GPIO_Port GPIOA
#define M1_OCP_Pin LL_GPIO_PIN_6
#define M1_OCP_GPIO_Port GPIOA
#define M1_PWM_UL_Pin LL_GPIO_PIN_7
#define M1_PWM_UL_GPIO_Port GPIOA
#define M1_PWM_VL_Pin LL_GPIO_PIN_0
#define M1_PWM_VL_GPIO_Port GPIOB
#define M1_PWM_WL_Pin LL_GPIO_PIN_1
#define M1_PWM_WL_GPIO_Port GPIOB
#define M1_PWM_UH_Pin LL_GPIO_PIN_8
#define M1_PWM_UH_GPIO_Port GPIOA
#define M1_PWM_VH_Pin LL_GPIO_PIN_9
#define M1_PWM_VH_GPIO_Port GPIOA
#define M1_PWM_WH_Pin LL_GPIO_PIN_10
#define M1_PWM_WH_GPIO_Port GPIOA
#define TMS_Pin LL_GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin LL_GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define M1_ENCODER_A_Pin LL_GPIO_PIN_15
#define M1_ENCODER_A_GPIO_Port GPIOA
#define M1_ENCODER_B_Pin LL_GPIO_PIN_3
#define M1_ENCODER_B_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
