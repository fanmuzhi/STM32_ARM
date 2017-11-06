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
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//typedef struct{
//	TIM_HandleTypeDef *tim_handle;
//	uint32_t					tim_channel;
//}led_Timer_t;

//led_Timer_t led_R;
//led_Timer_t led_G;
//led_Timer_t led_B;
//led_Timer_t led_W;
//led_Timer_t led_IR;

typedef struct{
	TIM_HandleTypeDef *gpio_x;
	uint32_t					gpio_pin;
}RGB_led_Gpio_t;

led_Gpio_t led_R;
led_Gpio_t led_G;
led_Gpio_t led_B;
uint8_t UserTxBuffer[] = "---K1 button clicked--- \n";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_I2C2_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

//void vTurnOnLed( unsigned char ucLightness );
void vTurnOnLed( led_Timer_t *ledTimer, unsigned char ucLightness );
void vTurnOffLed( led_Timer_t *ledTimer);
//void vTurnOnRedLight( unsigned char ucPulseLenth );
//void vTurnOnGreenLight( unsigned char ucPulseLenth );
//void vTurnOnBlueLight( unsigned char ucPulseLenth );
//void vTurnOffRedLight(void);
//void vTurnOffGreenLight(void);
//void vTurnOffBlueLight(void);
int8_t VCP_CMD_process( void );
//extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
extern s_RxBuff_t s_RxBuff;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	led_R.tim_handle = &htim10;
	led_R.tim_channel = TIM_CHANNEL_1;
	led_G.tim_handle = &htim11;
	led_G.tim_channel = TIM_CHANNEL_1;
	led_B.tim_handle = &htim13;
	led_B.tim_channel = TIM_CHANNEL_1;
	led_W.tim_handle = &htim4;
	led_W.tim_channel = TIM_CHANNEL_3;
	led_IR.tim_handle = &htim4;
	led_IR.tim_channel = TIM_CHANNEL_4;
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

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t adc1_val, adc2_val;
	HAL_ADC_Start(&hadc1);
	adc1_val = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Start(&hadc2);
	adc2_val = HAL_ADC_GetValue(&hadc2);
	vTurnOnLed(&led_W, 100);
	vTurnOnLed(&led_IR, 125);
	vTurnOffLed(&led_W);
	vTurnOffLed(&led_IR);
	HAL_ADC_Stop(&hadc1);
	HAL_ADC_Stop(&hadc2);
	HAL_Delay(100);

  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

//		uint8_t *hexbuffer;
//		uint32_t pos32;
//		char *textbuf;
		int i_temp0 = 0;

		VCP_CMD_process( );
	
		HAL_Delay(10);//it is better to delay a little bit time, 0.1ms up to a few ms
		i_temp0++;
		vTurnOnLed(&led_R, 255);
		HAL_Delay(100);
		vTurnOffLed(&led_R);
		HAL_Delay(100);
		//CDC_Transmit_FS( UserTxBuffer,  sizeof(UserTxBuffer) );

		uint8_t pTxData[4] = {0x11,0x22,0x33,0x44};
		uint8_t pRxData[8];
 		HAL_SPI_TransmitReceive(&hspi1, pTxData, pRxData, 8, 2000);
	
		uint8_t cmd[5] = {0x11,0x22,0x33,0x44, 0xff};
		if(HAL_SPI_Transmit(&hspi1, cmd, 5, 1000) == HAL_OK)
		{
			
		}
		uint8_t pRxData2[5];
		if(HAL_SPI_Receive(&hspi1, pRxData2, 5, 1000) == HAL_OK)
		{
			HAL_Delay(100);
		}
  }
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

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 255;
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
  sConfigOC.Pulse = 25;
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
  htim11.Init.Period = 0;
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
  HAL_GPIO_WritePin(GPIOC, SPIVCC_200mV_Pin|SPIVCC_100mV_Pin|SPIVCC_50mV_Pin|SPIVCC_800mV_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPIVCC_400mV_Pin|SPIVCC_1600mV_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : SPIVCC_200mV_Pin SPIVCC_100mV_Pin SPIVCC_50mV_Pin SPIVCC_800mV_Pin */
  GPIO_InitStruct.Pin = SPIVCC_200mV_Pin|SPIVCC_100mV_Pin|SPIVCC_50mV_Pin|SPIVCC_800mV_Pin;
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

  /*Configure GPIO pins : SPIVCC_400mV_Pin SPIVCC_1600mV_Pin */
  GPIO_InitStruct.Pin = SPIVCC_400mV_Pin|SPIVCC_1600mV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : R_Pin G_Pin B_Pin */
  GPIO_InitStruct.Pin = R_Pin|G_Pin|B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : CALIBRATE_LOW_Pin */
  GPIO_InitStruct.Pin = CALIBRATE_LOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CALIBRATE_LOW_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint32_t GetLightness( void )
{
	uint32_t adc1_val, adc2_val;
	
	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start(&hadc2);
	adc1_val = HAL_ADC_GetValue(&hadc1);
	adc2_val = HAL_ADC_GetValue(&hadc2);
	return adc2_val;
}		

int8_t VCP_CMD_process()
{
    uint32_t lightness;
	
    if( s_RxBuff.IsCommandDataReceived == 0 )
		{
			return 0; //no data received
		}
    else
		{
			switch(s_RxBuff.UserRxBufferFS[0])
			{ 
				case SET_LED_WHITE:
//						vTurnOnRedLight(s_RxBuff.UserRxBufferFS[1]);
					break;
				
				case GET_LED_WHITE:
						lightness = GetLightness();
						CDC_Transmit_FS( (unsigned char*)(&lightness), sizeof(lightness) );      //send the text to PC via cdc
					break;
				
				case SET_LED_IR:
//						vTurnOnBlueLight(s_RxBuff.UserRxBufferFS[1]);
					break;
				
				case GET_LED_IR:
						lightness = GetLightness();
						CDC_Transmit_FS( (unsigned char*)(&lightness), sizeof(lightness) );      //send the text to PC via cdc
					break;
				
				default:
					
					break;
			}
			s_RxBuff.IsCommandDataReceived = 0;
			
		}
		
//    //check if all data were processed.
//    s_RxBuffers.pos_process++;
//    if(s_RxBuffers.pos_process>=MaxCommandsInBuffer) //reach the last buffer, need to rewind to 0
//    {
//        s_RxBuffers.pos_process=0;
//    }
//    if(s_RxBuffers.pos_process==s_RxBuffers.pos_receive)s_RxBuffers.IsCommandDataReceived=0; //check if all data were processed
//		
    return 1;
}

void vTurnOnLed( led_Timer_t *ledTimer, unsigned char ucLightness )
{
  TIM_OC_InitTypeDef sConfigOC;
               
	if(HAL_TIM_PWM_Stop(ledTimer->tim_handle, ledTimer->tim_channel) != HAL_OK)
	{
		/* PWM Generation Error */
		//Error_Handler();
	}
               
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = ( uint32_t )( 255 - ucLightness );
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(ledTimer->tim_handle, &sConfigOC, ledTimer->tim_channel);
               
	if(HAL_TIM_PWM_Start(ledTimer->tim_handle, ledTimer->tim_channel) != HAL_OK)
	{
		/* PWM Generation Error */
		//Error_Handler();
	}              
}
void vTurnOffLed( led_Timer_t *ledTimer)
{               
	if(HAL_TIM_PWM_Start(ledTimer->tim_handle, ledTimer->tim_channel) == HAL_OK)
	{
		HAL_TIM_PWM_Stop(ledTimer->tim_handle, ledTimer->tim_channel);
	}              
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_0)
  {
    /* Toggle LED1 */
    vTurnOffLed(&led_R);
		CDC_Transmit_FS( UserTxBuffer,  sizeof(UserTxBuffer) );
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
