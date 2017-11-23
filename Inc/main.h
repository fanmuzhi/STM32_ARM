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
#include "vcsTypes.h"
#include "vcsfw_v4.h"

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

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
// PC5		PC4		PC3		PC2		PC1		PC0
// 1.6V		0.4V	0.8V	0.05V	0.1V	0.2V
// Vout = 0.5 + PCx
#define SPIVCC_0V00      0x3F
#define SPIVCC_0V90      0x2F
#define SPIVCC_0V95      0x2B
#define SPIVCC_1V00      0x2D
#define SPIVCC_1V05      0x29
#define SPIVCC_1V10      0x2E
#define SPIVCC_1V15      0x2A
#define SPIVCC_1V20      0x2C
#define SPIVCC_1V25      0x28
#define SPIVCC_1V30      0x37
#define SPIVCC_1V35      0x33
#define SPIVCC_1V40      0x35
#define SPIVCC_1V45      0x31
#define SPIVCC_1V50      0x36
#define SPIVCC_1V55      0x32
#define SPIVCC_1V60      0x34
#define SPIVCC_1V65      0x30
#define SPIVCC_1V70      0x27
#define SPIVCC_1V75      0x23
#define SPIVCC_1V80      0x25
#define SPIVCC_1V85      0x21
#define SPIVCC_1V90      0x26
#define SPIVCC_1V95      0x22
#define SPIVCC_2V00      0x24
#define SPIVCC_2V05      0x20
#define SPIVCC_2V10      0x1F
#define SPIVCC_2V15      0x1B
#define SPIVCC_2V20      0x1D
#define SPIVCC_2V25      0x19
#define SPIVCC_2V30      0x1E
#define SPIVCC_2V35      0x1A
#define SPIVCC_2V40      0x1C
#define SPIVCC_2V45      0x18
#define SPIVCC_2V50      0x0F
#define SPIVCC_2V55      0x0B
#define SPIVCC_2V60      0x0D
#define SPIVCC_2V65      0x09
#define SPIVCC_2V70      0x0E
#define SPIVCC_2V75      0x0A
#define SPIVCC_2V80      0x0C
#define SPIVCC_2V85      0x08
#define SPIVCC_2V90      0x17
#define SPIVCC_2V95      0x13
#define SPIVCC_3V00      0x15
#define SPIVCC_3V05      0x11
#define SPIVCC_3V10      0x16
#define SPIVCC_3V15      0x12
#define SPIVCC_3V20      0x14
#define SPIVCC_3V25      0x10
#define SPIVCC_3V30      0x07
#define SPIVCC_3V35      0x03
#define SPIVCC_3V40      0x05
#define SPIVCC_3V45      0x01
#define SPIVCC_3V50      0x06

// PE5		PE4		PE3		PE2		PE1		PE0
// 1.6V		0.8V	0.4V	0.2V	0.1V	0.05V
// Vout = 0.5 + PEx
#define SENSOR_VCC_0V00         0x3F
#define SENSOR_VCC_0V90         0x37
#define SENSOR_VCC_0V95         0x36
#define SENSOR_VCC_1V00         0x35
#define SENSOR_VCC_1V05         0x34
#define SENSOR_VCC_1V10         0x33
#define SENSOR_VCC_1V15         0x32
#define SENSOR_VCC_1V20         0x31
#define SENSOR_VCC_1V25         0x30
#define SENSOR_VCC_1V30         0x2F
#define SENSOR_VCC_1V35         0x2E
#define SENSOR_VCC_1V40         0x2D
#define SENSOR_VCC_1V45         0x2C
#define SENSOR_VCC_1V50         0x2B
#define SENSOR_VCC_1V55         0x2A
#define SENSOR_VCC_1V60         0x29
#define SENSOR_VCC_1V65         0x28
#define SENSOR_VCC_1V70         0x27
#define SENSOR_VCC_1V75         0x26
#define SENSOR_VCC_1V80         0x25
#define SENSOR_VCC_1V85         0x24
#define SENSOR_VCC_1V90         0x23
#define SENSOR_VCC_1V95         0x22
#define SENSOR_VCC_2V00         0x21
#define SENSOR_VCC_2V05         0x20
#define SENSOR_VCC_2V10         0x1F
#define SENSOR_VCC_2V15         0x1E
#define SENSOR_VCC_2V20         0x1D
#define SENSOR_VCC_2V25         0x1C
#define SENSOR_VCC_2V30         0x1B
#define SENSOR_VCC_2V35         0x1A
#define SENSOR_VCC_2V40         0x19
#define SENSOR_VCC_2V45         0x18
#define SENSOR_VCC_2V50         0x17
#define SENSOR_VCC_2V55         0x16
#define SENSOR_VCC_2V60         0x15
#define SENSOR_VCC_2V65         0x14
#define SENSOR_VCC_2V70         0x13
#define SENSOR_VCC_2V75         0x12
#define SENSOR_VCC_2V80         0x11
#define SENSOR_VCC_2V85         0x10
#define SENSOR_VCC_2V90         0x0F
#define SENSOR_VCC_2V95         0x0E
#define SENSOR_VCC_3V00         0x0D
#define SENSOR_VCC_3V05         0x0C
#define SENSOR_VCC_3V10         0x0B
#define SENSOR_VCC_3V15         0x0A
#define SENSOR_VCC_3V20         0x09
#define SENSOR_VCC_3V25         0x08
#define SENSOR_VCC_3V30         0x07
#define SENSOR_VCC_3V35         0x06
#define SENSOR_VCC_3V40         0x05
#define SENSOR_VCC_3V45         0x04
#define SENSOR_VCC_3V50         0x03

//#define LED_WHITE									0
//#define	LED_IR    								1
#define VCP_CMD_SET_LED_DN				0x10
#define VCP_CMD_SET_LED_TIM				0x11
#define VCP_CMD_GET_LED_DN 				0x20
#define VCP_CMD_FP_GETVER					0x50
#define LED_MIN 									0x0000
#define LED_MAX 									0x0fff
#define DN_LED_MIN 								0x0000
#define DN_LED_MAX 								0x0fff
//#define TARGET_LIGHT_DN_VAL			3000
#define TIME_LED_DELAY	 					100			//	ms, time after waiting the LED intensity stabal after light on
#define LIGHT_AUTOADJUST_TIME_MAX	20

#define EP0SIZE_BIG                8
#define EPSELBYTE_INTEGRIFY(x)      \
    (((x)& ~0xe0) | (((~(x)& 1) << 7) | ((~(x)& 2) << 5) | ((~(x)& 4) << 3)))
#define EPSELBYTE_EP0IN     ((0 << 1) | 1)
/*
 * Fields in the EP0IN register.
 */
#define EP0IN_SOFTSTATE           0x0000001f
#define EP0IN_SOFTSTATE_B             0
#define EP0IN_SOFTSTATE_N             5
#define EP0IN_SOFTSTATE_EP1STATE  0x00000003
#define EP0IN_SOFTSTATE_EP1STATE_B    0
#define EP0IN_SOFTSTATE_EP1STATE_N    2
#define EP0IN_SOFTSTATE_EP1STATE_OFF        0
#define EP0IN_SOFTSTATE_EP1STATE_REPLYSENT  0
#define EP0IN_SOFTSTATE_EP1STATE_CMDWAIT    1
#define EP0IN_SOFTSTATE_EP1STATE_CMDPROC    2
#define EP0IN_SOFTSTATE_EP1STATE_REPLY      3
#define EP0IN_SOFTSTATE_FPSTATE   0x0000000c
#define EP0IN_SOFTSTATE_FPSTATE_B     2
#define EP0IN_SOFTSTATE_FPSTATE_N     2
#define EP0IN_SOFTSTATE_FPSTATE_UNKNOWN     0
#define EP0IN_SOFTSTATE_FPSTATE_ABSENT      1
#define EP0IN_SOFTSTATE_FPSTATE_STILL       2
#define EP0IN_SOFTSTATE_FPSTATE_MOVING      3
#define EP0IN_SOFTSTATE_SSLSTATE  0x00000010
#define EP0IN_SOFTSTATE_SSLSTATE_B    4
#define EP0IN_SOFTSTATE_SSLSTATE_N    1
#define EP0IN_EP2FLUSH            0x00000020
#define EP0IN_EP2FLUSH_B              5
#define EP0IN_JUSTWOKE            0x00000040
#define EP0IN_JUSTWOKE_B              6
#define EP0IN_STATECHANGED        0x00000080
#define EP0IN_STATECHANGED_B          7
#define EP0IN_EP2INSIZE           0x01ffff00
#define EP0IN_EP2INSIZE_B             8
#define EP0IN_EP2INSIZE_N             17
#define EP0IN_EP2INDONE           0x02000000
#define EP0IN_EP2INDONE_B             25
#define EP0IN_RUNNING             0x04000000
#define EP0IN_RUNNING_B               26
#define EP0IN_EP1OUT              0x08000000
#define EP0IN_EP1OUT_B                27
#define EP0IN_EP1IN               0x10000000
#define EP0IN_EP1IN_B                 28
#define EP0IN_DRDY                0x20000000
#define EP0IN_DRDY_B                  29
#define EP0IN_JUSTRESET           0x40000000
#define EP0IN_JUSTRESET_B             30
#define EP0IN_ALIVE               0x80000000
#define EP0IN_ALIVE_B                 31
/*
 * The high bits of EP0IN (EP0_IN_SPI_I2C_STS_1)
 */
#define EP0IN_SOFTSTATE2          0xffff000000000000ULL
#define EP0IN_SOFTSTATE2_B        48
#define EP0IN_SOFTSTATE2_N        16
#define EP0IN_EP1INSIZE           0x0000ffff00000000ULL
#define EP0IN_EP1INSIZE_B         32
#define EP0IN_EP1INSIZE_N         16

/*
 * The end points supported on SPI interface
 */
#define EPSELBYTE_EP0IN     ((0 << 1) | 1)
#define EPSELBYTE_EP1OUT    ((1 << 1) | 0)
#define EPSELBYTE_EP1IN     ((1 << 1) | 1)
#define EPSELBYTE_EP2IN     ((2 << 1) | 1)
#define EPSELBYTE_EP3OUT    ((3 << 1) | 0)
#define EPSELBYTE_RESET     0xf0
#define EPSELBYTE_ENTERSLEEP        0xE8
#define EPSELBYTE_EXITSLEEP         0x17

#define EP0SIZE                    4
#define EP0SIZE_BIG                8


/*
 * Mask a 64-bit value and produce a 32-bit one.
 */
#define EXTRACT32(val, mask, shift)                                         \
    (unsigned int)(((val)& (mask)) >> (shift))

/* Given a 32-bit value, swap the bytes */
#define ENDIANSWAP32(val)                                                   \
    ((((val)& 0xff) << 24)                                                 \
    | ((((val) >> 8) & 0xff) << 16)                                        \
    | ((((val) >> 16) & 0xff) << 8)                                        \
    | (((val) >> 24) & 0xff))

/* Convert a known big-endian value to host (little) endian value */
#define BIGTOHOST32(valp)                                                   \
    ENDIANSWAP32(*((const uint32_t *)(valp)))

/* Convert a local 32-bit value to a big endian value */
#define HOSTTOBIG32(destp, val) (*((uint32_t *) (destp)) = ENDIANSWAP32(val))
		
// exception error code definitions
#define ERROR_NONE											0x00000000
#define ERROR_SET_LED_EXCEEDED					0xF0000001
#define ERROR_SET_DN_EXCEEDED						0xF0000002
#define ERROR_ADJUST_TO_TARGET_FAIL			0xF0000003

//error code
#define ERROR_TIME_OUT					0x1500
#define ERROR_CRC_VERIFY				0x1501
#define ERROR_BL_MODE					0x1502
#define ERROR_PARAM_UNDEFINE			0x1503
#define ERROR_TYPE						0x1504
#define ERROR_PARAMETER					0x1505
#define ERROR_RESULT_NOTFOUND			0x1506
#define ERROR_REPLY_LENGTH_TOO_SHORT	0x1507
		
		
//#define assertmcs(Gpio_PortPin_t *mcs_pin)		HAL_GPIO_WritePin(mcs_pi, SPI5_CS_Pin, GPIO_PIN_RESET)		
//#define deassertmcs(*Gpio_PortPin_t) HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_SET)
		
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
