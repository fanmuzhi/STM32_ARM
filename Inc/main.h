/**
  ******************************************************************************
  * File Name          : main.h
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

/* USER CODE BEGIN Includes */

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
#define SPIVCC_200mV_Pin GPIO_PIN_0
#define SPIVCC_200mV_GPIO_Port GPIOC
#define SPIVCC_100mV_Pin GPIO_PIN_1
#define SPIVCC_100mV_GPIO_Port GPIOC
#define SPIVCC_50mV_Pin GPIO_PIN_2
#define SPIVCC_50mV_GPIO_Port GPIOC
#define SPIVCC_800mV_Pin GPIO_PIN_3
#define SPIVCC_800mV_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_4
#define BUZZER_GPIO_Port GPIOH
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

/* USER CODE BEGIN Private defines */
#define LED_WHITE					0
#define	LED_IR    				1
#define VCP_CMD_SET_LED_DN				0x10
#define VCP_CMD_SET_LED_TIM				0x11
#define VCP_CMD_GET_LED_DN 					0x20

#define TARGET_LIGHT_VAL			3000
#define TIME_LED_DELAY	 	100			//	ms, time after waiting the LED intensity stabal after light on

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
