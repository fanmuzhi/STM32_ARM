/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */


//#pragma pack(8)
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define VCC_200mV_Pin GPIO_PIN_2
#define VCC_200mV_GPIO_Port GPIOE
#define VCC_400mV_Pin GPIO_PIN_3
#define VCC_400mV_GPIO_Port GPIOE
#define VCC_800mV_Pin GPIO_PIN_4
#define VCC_800mV_GPIO_Port GPIOE
#define VCC_1600mV_Pin GPIO_PIN_5
#define VCC_1600mV_GPIO_Port GPIOE
#define VCC_PG_Pin GPIO_PIN_8
#define VCC_PG_GPIO_Port GPIOI
#define SPIVCC_PG_Pin GPIO_PIN_9
#define SPIVCC_PG_GPIO_Port GPIOI
#define VCC_EN_Pin GPIO_PIN_10
#define VCC_EN_GPIO_Port GPIOI
#define SPIVCC_EN_Pin GPIO_PIN_11
#define SPIVCC_EN_GPIO_Port GPIOI
#define SPI5_CS_Pin GPIO_PIN_6
#define SPI5_CS_GPIO_Port GPIOF
#define SPIVCC_200mV_Pin GPIO_PIN_0
#define SPIVCC_200mV_GPIO_Port GPIOC
#define SPIVCC_100mV_Pin GPIO_PIN_1
#define SPIVCC_100mV_GPIO_Port GPIOC
#define SPIVCC_50mV_Pin GPIO_PIN_2
#define SPIVCC_50mV_GPIO_Port GPIOC
#define SPIVCC_800mV_Pin GPIO_PIN_3
#define SPIVCC_800mV_GPIO_Port GPIOC
#define RANGE_SEL_Pin GPIO_PIN_3
#define RANGE_SEL_GPIO_Port GPIOH
#define BUZZER_Pin GPIO_PIN_4
#define BUZZER_GPIO_Port GPIOH
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define SPIVCC_400mV_Pin GPIO_PIN_4
#define SPIVCC_400mV_GPIO_Port GPIOC
#define SPIVCC_1600mV_Pin GPIO_PIN_5
#define SPIVCC_1600mV_GPIO_Port GPIOC
#define R_Pin GPIO_PIN_2
#define R_GPIO_Port GPIOG
#define G_Pin GPIO_PIN_3
#define G_GPIO_Port GPIOG
#define B_Pin GPIO_PIN_4
#define B_GPIO_Port GPIOG
#define KEY2_Pin GPIO_PIN_9
#define KEY2_GPIO_Port GPIOC
#define KEY2_EXTI_IRQn EXTI9_5_IRQn
#define KEY1_Pin GPIO_PIN_1
#define KEY1_GPIO_Port GPIOI
#define KEY1_EXTI_IRQn EXTI1_IRQn
#define VCC_50mV_Pin GPIO_PIN_0
#define VCC_50mV_GPIO_Port GPIOE
#define VCC_100mV_Pin GPIO_PIN_1
#define VCC_100mV_GPIO_Port GPIOE
#define CALIBRATE_LOW_Pin GPIO_PIN_7
#define CALIBRATE_LOW_GPIO_Port GPIOI

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */


//#define assertmcs()		HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_RESET)		
//#define deassertmcs() HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_SET)
		
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
