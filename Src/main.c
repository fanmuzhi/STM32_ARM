/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#pragma pack(8)
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

typedef struct{
								GPIO_TypeDef 		*gpio_x;
								uint16_t				gpio_pin;
}Gpio_RGB_Led_t;

typedef struct{
								TIM_HandleTypeDef		*tim_handle;
								uint32_t						tim_channel;
								uint16_t						tim_started;
								uint16_t						last_led_tim_val;
}Timer_Led_t;

typedef struct{
								uint16_t						result_dn;
								uint16_t						gaugeMinScale;
								ADC_HandleTypeDef		*gauge_handle;
}Lightness_Gauge_t;

typedef struct{
								uint32_t	error_LightW;
								uint32_t	error_LightIR;
								uint16_t	lightnessWDN;
								uint16_t	lightnessIRDN;
}VCP_Light_Return_t;

Timer_Led_t led_W		= {	.tim_handle = &htim10, 
												.tim_channel = TIM_CHANNEL_1, 
												.tim_started = 0, 
												.last_led_tim_val = 0};

Timer_Led_t led_IR	= {	.tim_handle = &htim11, 
												.tim_channel = TIM_CHANNEL_1, 
												.tim_started = 0, 
												.last_led_tim_val = 0 };

Lightness_Gauge_t lightnessGauge	=	{	.gauge_handle = &hadc3, 
																			.result_dn = 0x0000, 
																			.gaugeMinScale = 0x0C };

VCP_Light_Return_t vcp_lightdn_ret	=	{	.error_LightW = 0xffffffff, 
																				.error_LightIR = 0xffffffff, 
																				.lightnessWDN = 0xffff, 
																				.lightnessIRDN = 0xffff };

Gpio_RGB_Led_t led_R = {.gpio_x = GPIOG, 
												.gpio_pin = GPIO_PIN_2};

Gpio_RGB_Led_t led_G = {.gpio_x = GPIOG, 
												.gpio_pin = GPIO_PIN_3};

Gpio_RGB_Led_t led_B = {.gpio_x = GPIOG, 
												.gpio_pin = GPIO_PIN_4};

uint8_t UserTxBuffer[] = "---K1 button clicked--- \n";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI5_Init(void);
static void MX_SPI1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void vTurnOnRgbLed( uint16_t r, uint16_t g, uint16_t b );
void vTurnOffRgbLed(void);
	
uint16_t vTurnOnLed( Timer_Led_t *ledTimer, uint16_t ucLightness );
uint16_t vTurnOffLed( Timer_Led_t *ledTimer);
uint16_t GetLightness( Lightness_Gauge_t *lightnessGauge );
int8_t VCP_CMD_process( void );
int32_t AutoAdjustLed(Timer_Led_t *ledTimer, Lightness_Gauge_t *lightGauge, uint16_t dn_target);

int iEnableSPIVCC( int iVoltage );
int iEnableSENSORVCC( int iVoltage );
//extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
extern s_RxBuff_t s_RxBuff;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USB_DEVICE_Init();
  MX_ADC3_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_I2C2_Init();
  MX_SPI5_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
	iEnableSPIVCC( SPIVCC_1V80 );
	iEnableSENSORVCC( SENSOR_VCC_2V80 ) ;
	HAL_Delay(10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		VCP_CMD_process( );
//		uint8_t pTxData[4] = {0x01,0xff,0xff,0xff};
//		uint8_t pRxData[8];
// 		HAL_SPI_TransmitReceive(&hspi1, pTxData, pRxData, 8, 2000);
//	
//		uint8_t cmd[5] = {0x11,0x22,0x33,0x44, 0xff};
//		if(HAL_SPI_Transmit(&hspi1, cmd, 5, 1000) == HAL_OK)
//		{
//			
//		}
//		uint8_t pRxData2[5];
//		if(HAL_SPI_Receive(&hspi1, pRxData2, 5, 1000) == HAL_OK)
//		{
//			HAL_Delay(100);
//		}
	}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC3 init function */
static void MX_ADC3_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI5 init function */
static void MX_SPI5_Init(void)
{

  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 4095;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim10);

}

/* TIM11 init function */
static void MX_TIM11_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 4095;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim11);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, VCC_200mV_Pin|VCC_400mV_Pin|VCC_800mV_Pin|VCC_1600mV_Pin 
                          |GPIO_PIN_6|VCC_50mV_Pin|VCC_100mV_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, VCC_EN_Pin|SPIVCC_EN_Pin|CALIBRATE_LOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SPIVCC_200mV_Pin|SPIVCC_100mV_Pin|SPIVCC_50mV_Pin|SPIVCC_800mV_Pin 
                          |SPIVCC_400mV_Pin|SPIVCC_1600mV_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, R_Pin|G_Pin|B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : VCC_200mV_Pin VCC_400mV_Pin VCC_800mV_Pin VCC_1600mV_Pin 
                           PE6 VCC_50mV_Pin VCC_100mV_Pin */
  GPIO_InitStruct.Pin = VCC_200mV_Pin|VCC_400mV_Pin|VCC_800mV_Pin|VCC_1600mV_Pin 
                          |GPIO_PIN_6|VCC_50mV_Pin|VCC_100mV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : VCC_PG_Pin SPIVCC_PG_Pin */
  GPIO_InitStruct.Pin = VCC_PG_Pin|SPIVCC_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : VCC_EN_Pin SPIVCC_EN_Pin */
  GPIO_InitStruct.Pin = VCC_EN_Pin|SPIVCC_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : SPIVCC_200mV_Pin SPIVCC_100mV_Pin SPIVCC_50mV_Pin SPIVCC_800mV_Pin 
                           SPIVCC_400mV_Pin SPIVCC_1600mV_Pin */
  GPIO_InitStruct.Pin = SPIVCC_200mV_Pin|SPIVCC_100mV_Pin|SPIVCC_50mV_Pin|SPIVCC_800mV_Pin 
                          |SPIVCC_400mV_Pin|SPIVCC_1600mV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R_Pin G_Pin B_Pin */
  GPIO_InitStruct.Pin = R_Pin|G_Pin|B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY2_Pin */
  GPIO_InitStruct.Pin = KEY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY1_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CALIBRATE_LOW_Pin */
  GPIO_InitStruct.Pin = CALIBRATE_LOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CALIBRATE_LOW_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
uint16_t GetLightness( Lightness_Gauge_t *lightnessGauge )
{

	lightnessGauge->gauge_handle->ErrorCode = 0xff;
	lightnessGauge->result_dn = 0xffff;
	HAL_ADC_Start(lightnessGauge->gauge_handle);
	HAL_ADC_PollForConversion(lightnessGauge->gauge_handle, 2000);
	lightnessGauge->gauge_handle->ErrorCode = lightnessGauge->gauge_handle->ErrorCode;
	lightnessGauge->result_dn = HAL_ADC_GetValue(lightnessGauge->gauge_handle);
	return 0;
}		

/* VCP command processing function*/
int8_t VCP_CMD_process()
{
    if( s_RxBuff.IsCommandDataReceived == 0 )
		{
			return 0; //no data received
		}
    else
		{
			switch(s_RxBuff.UserRxBufferFS[0])
			{
				// process cmd set led by digital number needed, and return light digital number
				case VCP_CMD_SET_LED_DN:
				{
					vcp_lightdn_ret.error_LightW	= 0xffffffff, 
					vcp_lightdn_ret.error_LightIR	= 0xffffffff, 
					vcp_lightdn_ret.lightnessWDN	= 0xffff;
					vcp_lightdn_ret.lightnessIRDN	= 0xffff;
					if(s_RxBuff.UserRxBufferFS[2] >= 0x10 || s_RxBuff.UserRxBufferFS[4] >= 0x10)
					{
						CDC_Transmit_FS((uint8_t *)(&vcp_lightdn_ret), sizeof(vcp_lightdn_ret));      //send back the detected light adc value
						break;
					}
					uint16_t lightnessW_target = (uint16_t)s_RxBuff.UserRxBufferFS[1] + (((uint16_t)s_RxBuff.UserRxBufferFS[2])<<8);
					uint16_t lightnessIR_target = (uint16_t)s_RxBuff.UserRxBufferFS[3] + (((uint16_t)s_RxBuff.UserRxBufferFS[4])<<8);
					
					//adjust IR Led to lightnessIR_target
					vTurnOffLed(&led_W);
//				vTurnOffLed(&led_IR);
					AutoAdjustLed(&led_IR, &lightnessGauge, lightnessIR_target);
					vcp_lightdn_ret.error_LightIR = lightnessGauge.gauge_handle->ErrorCode;
					vcp_lightdn_ret.lightnessIRDN = lightnessGauge.result_dn;
					
					//adjust W Led to lightness_target
//				vTurnOffLed(&led_W);
					vTurnOffLed(&led_IR);
					AutoAdjustLed(&led_W, &lightnessGauge, lightnessW_target);
					vcp_lightdn_ret.error_LightW = lightnessGauge.gauge_handle->ErrorCode;
					vcp_lightdn_ret.lightnessWDN = lightnessGauge.result_dn;
					
					vTurnOnLed(&led_W, led_W.last_led_tim_val);		//re-light on IR LED again 
					vTurnOnLed(&led_IR, led_IR.last_led_tim_val);		//re-light on IR LED again 

					CDC_Transmit_FS((uint8_t *)(&vcp_lightdn_ret), sizeof(vcp_lightdn_ret));      //send back the detected light adc value
					break;
				}
				
				//process cmd set led lightness by timer number, and return light digital number
				case VCP_CMD_SET_LED_TIM:
				{
					uint16_t vcp_ret_get_lightDN[2] = {0xffff, 0xffff};
					uint16_t ledW_level = (uint16_t)s_RxBuff.UserRxBufferFS[1] + (((uint16_t)s_RxBuff.UserRxBufferFS[2])<<8);
					uint16_t ledIR_level = (uint16_t)s_RxBuff.UserRxBufferFS[3] + (((uint16_t)s_RxBuff.UserRxBufferFS[4])<<8);
					vTurnOnLed(&led_W, ledW_level);
					vTurnOnLed(&led_IR, ledIR_level);
					HAL_Delay(100);
					GetLightness(&lightnessGauge);
					vcp_ret_get_lightDN[0] = lightnessGauge.gauge_handle->ErrorCode;
					vcp_ret_get_lightDN[1] = lightnessGauge.result_dn;
					CDC_Transmit_FS( (uint8_t *)(&vcp_ret_get_lightDN), sizeof(vcp_ret_get_lightDN));       //send back the detected light adc value
					break;
				}
				// process cmd get led digtal number back , and return light digital number
				case VCP_CMD_GET_LED_DN:
				{
					uint16_t vcp_ret_get_lightDN[2] = {0xffff, 0xffff};
					GetLightness(&lightnessGauge);
					vcp_ret_get_lightDN[0] = lightnessGauge.gauge_handle->ErrorCode;
					vcp_ret_get_lightDN[1] = lightnessGauge.result_dn;
					CDC_Transmit_FS( (uint8_t *)(&vcp_ret_get_lightDN), sizeof(vcp_ret_get_lightDN));       //send back the detected light adc value
					break;
				}
				
				default:	
					break;
			}
			s_RxBuff.IsCommandDataReceived = 0;
		}
    return 1;
}

//function to auto adjust Led to the dn_target value
int32_t AutoAdjustLed(Timer_Led_t *ledTimer, Lightness_Gauge_t *lightGauge, uint16_t dn_target)
{
	if(dn_target == DN_LED_MIN || dn_target == DN_LED_MAX)
	{
		vTurnOnLed(ledTimer, dn_target);
		GetLightness(lightGauge);
		if (abs(lightGauge->result_dn - dn_target) > lightGauge->gaugeMinScale)
		{
			lightnessGauge.gauge_handle->ErrorCode = 0xffff;          // Led cannot be adjusted to needed value, value should be out of range.
			lightnessGauge.result_dn = lightGauge->result_dn;
			return -1;
		}
		else
			{
		return 0;
		}
	}
	uint16_t led_low = LED_MIN, led_high = LED_MAX;
	uint8_t times = 0;	
	while (led_low <= led_high) 
	{
		times++;
		if(times >= LIGHT_AUTOADJUST_TIME_MAX)
		{
			lightnessGauge.gauge_handle->ErrorCode = 0xffff;          // Led cannot be adjusted to needed value, value should be out of range.
			return -1;
		}	
		uint16_t led_middle = (led_low + led_high) / 2;
		vTurnOnLed(ledTimer, led_middle);
		GetLightness(lightGauge);
		if (lightGauge->gauge_handle->ErrorCode != 0)
		{
			return -1;
		}
		if (abs(lightGauge->result_dn - dn_target) <= (lightGauge->gaugeMinScale)/2)
		{
			return 0;
		}
		else if (lightGauge->result_dn > dn_target + lightGauge->gaugeMinScale) 
		{
			led_high = led_middle;
		}
		else if(lightGauge->result_dn < dn_target - lightGauge->gaugeMinScale) 
		{
			led_low = led_middle;
		}
	}
	return -1;
}

void vTurnOnRgbLed( uint16_t r, uint16_t g, uint16_t b )
{
		HAL_GPIO_WritePin(led_R.gpio_x, led_R.gpio_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led_G.gpio_x, led_G.gpio_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led_B.gpio_x, led_B.gpio_pin, GPIO_PIN_SET);
}

void vTurnOffRgbLed(void)
{
		HAL_GPIO_WritePin(led_R.gpio_x, led_R.gpio_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led_G.gpio_x, led_G.gpio_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led_B.gpio_x, led_B.gpio_pin, GPIO_PIN_RESET);
}

uint16_t vTurnOnLed( Timer_Led_t *ledTimer, uint16_t ucLightness )
{
	if(ucLightness == 0x0000)
	{
		vTurnOffLed(ledTimer);
		return 0;
	}
	ucLightness = (ucLightness > 0xfff)?0xfff:ucLightness;

  TIM_OC_InitTypeDef sConfigOC;             
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = (uint32_t)ucLightness;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(ledTimer->tim_handle, &sConfigOC, ledTimer->tim_channel);
               
	if(HAL_TIM_PWM_Start(ledTimer->tim_handle, ledTimer->tim_channel) != HAL_OK)
	{
//		ledTimer->last_led_tim_val = 0xffff;
	}
	ledTimer->last_led_tim_val = ucLightness;
	ledTimer->tim_started = 1;
	HAL_Delay(200);
	return 0;
}

uint16_t vTurnOffLed( Timer_Led_t *ledTimer)
{    
	TIM_OC_InitTypeDef sConfigOC;             
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = (uint32_t)0x00000000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(ledTimer->tim_handle, &sConfigOC, ledTimer->tim_channel);
               
	if(HAL_TIM_PWM_Start(ledTimer->tim_handle, ledTimer->tim_channel) != HAL_OK)
	{
//		ledTimer->last_led_tim_val = 0xffff;
	}

	if(HAL_TIM_PWM_Stop(ledTimer->tim_handle, ledTimer->tim_channel) != HAL_OK)
	{
		
	}
	ledTimer->last_led_tim_val = 0;
	ledTimer->tim_started = 0;

	HAL_Delay(200);
	return 0;
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_1)
  {
    /* Switch LED White */
		if(led_W.tim_started != 0)
		{
			vTurnOffLed(&led_W);
		}
    else
		{
			vTurnOnLed(&led_W, 0x0fff);
		}
//		CDC_Transmit_FS( UserTxBuffer,  sizeof(UserTxBuffer) );
  }  
	else if(GPIO_Pin == GPIO_PIN_9)
  {
    /* Switch LED IR */
		if(led_IR.tim_started != 0)
		{
			vTurnOffLed(&led_IR);
		}
    else
		{
			vTurnOnLed(&led_IR, 0x0fff);
		}
//		CDC_Transmit_FS( UserTxBuffer,  sizeof(UserTxBuffer) );
  } 
}

// poweron SPIVCC with the voltage defined in head file macro
// Return value: 0 is success, 2 is failure
int iEnableSPIVCC( int iVoltage )
{
	 int count;
	 
	 HAL_GPIO_WritePin( GPIOC, SPIVCC_200mV_Pin,  iVoltage & SPIVCC_200mV_Pin  ? GPIO_PIN_SET : GPIO_PIN_RESET );      // set low to enable 200mV
	 HAL_GPIO_WritePin( GPIOC, SPIVCC_100mV_Pin,  iVoltage & SPIVCC_100mV_Pin  ? GPIO_PIN_SET : GPIO_PIN_RESET );      // set low to enable 100mV
	 HAL_GPIO_WritePin( GPIOC, SPIVCC_50mV_Pin,   iVoltage & SPIVCC_50mV_Pin   ? GPIO_PIN_SET : GPIO_PIN_RESET );      // set low to enable 50mV
	 HAL_GPIO_WritePin( GPIOC, SPIVCC_800mV_Pin,  iVoltage & SPIVCC_800mV_Pin  ? GPIO_PIN_SET : GPIO_PIN_RESET );      // set low to enable 800mV
	 HAL_GPIO_WritePin( GPIOC, SPIVCC_400mV_Pin,  iVoltage & SPIVCC_400mV_Pin  ? GPIO_PIN_SET : GPIO_PIN_RESET );      // set low to enable 400mV
	 HAL_GPIO_WritePin( GPIOC, SPIVCC_1600mV_Pin, iVoltage & SPIVCC_1600mV_Pin ? GPIO_PIN_SET : GPIO_PIN_RESET ); // set low to enable 1600mV
				 
	 HAL_GPIO_WritePin( GPIOI, SPIVCC_EN_Pin, GPIO_PIN_SET);            // enable SPIVCC
	 
	 count = 5000; // 2266 actual
	 while( ( HAL_GPIO_ReadPin( SPIVCC_PG_GPIO_Port, SPIVCC_PG_Pin ) != GPIO_PIN_SET ) && ( count != 0 ) )      // PG is set?
	 {
				 count--;
	 }
	 
	 if( count )
	 {
				 return 0;
	 }
	 else
	 {
				 return 2;
	 }
}

// poweron module VCC with the voltage defined in head file macro
// Return value: 0 is success, 1 is parameter out of range, 2 is failure
int iEnableSENSORVCC( int iVoltage )
{
	 int count;
	 
	 if( iVoltage > SENSOR_VCC_0V90 || iVoltage < SENSOR_VCC_3V50 )     // out of range
	 {
				 return 1;
	 }
	 
	 HAL_GPIO_WritePin( GPIOE, VCC_200mV_Pin,  iVoltage & VCC_200mV_Pin  ? GPIO_PIN_SET : GPIO_PIN_RESET ); // set low to enable 200mV
	 HAL_GPIO_WritePin( GPIOE, VCC_100mV_Pin,  iVoltage & VCC_100mV_Pin  ? GPIO_PIN_SET : GPIO_PIN_RESET ); // set low to enable 100mV
	 HAL_GPIO_WritePin( GPIOE, VCC_50mV_Pin,   iVoltage & VCC_50mV_Pin   ? GPIO_PIN_SET : GPIO_PIN_RESET ); // set low to enable 50mV
	 HAL_GPIO_WritePin( GPIOE, VCC_800mV_Pin,  iVoltage & VCC_800mV_Pin  ? GPIO_PIN_SET : GPIO_PIN_RESET ); // set low to enable 800mV
	 HAL_GPIO_WritePin( GPIOE, VCC_400mV_Pin,  iVoltage & VCC_400mV_Pin  ? GPIO_PIN_SET : GPIO_PIN_RESET ); // set low to enable 400mV
	 HAL_GPIO_WritePin( GPIOE, VCC_1600mV_Pin, iVoltage & VCC_1600mV_Pin ? GPIO_PIN_SET : GPIO_PIN_RESET ); // set low to enable 1600mV
				 
	 HAL_GPIO_WritePin( GPIOI, VCC_EN_Pin, GPIO_PIN_SET);        // enable module VCC
	 
	 count = 5000; // 2266 actual
	 while( HAL_GPIO_ReadPin( VCC_PG_GPIO_Port, VCC_PG_Pin ) != GPIO_PIN_SET && count != 0 )     // PG is set?
	 {
				 count--;
	 }
	 
	 if( count )
	 {
				 return 0;
	 }
	 else
	 {
				 return 2;
	 }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
