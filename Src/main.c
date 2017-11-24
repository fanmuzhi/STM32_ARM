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
//#pragma pack(8)
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

typedef enum { false, true }bool;

static const uint32_t crc32_table[256] = {
    0x00000000, 0x04c11db7, 0x09823b6e, 0x0d4326d9,  /* [  0..  3] */
    0x130476dc, 0x17c56b6b, 0x1a864db2, 0x1e475005,  /* [  4..  7] */
    0x2608edb8, 0x22c9f00f, 0x2f8ad6d6, 0x2b4bcb61,  /* [  8.. 11] */
    0x350c9b64, 0x31cd86d3, 0x3c8ea00a, 0x384fbdbd,  /* [ 12.. 15] */
    0x4c11db70, 0x48d0c6c7, 0x4593e01e, 0x4152fda9,  /* [ 16.. 19] */
    0x5f15adac, 0x5bd4b01b, 0x569796c2, 0x52568b75,  /* [ 20.. 23] */
    0x6a1936c8, 0x6ed82b7f, 0x639b0da6, 0x675a1011,  /* [ 24.. 27] */
    0x791d4014, 0x7ddc5da3, 0x709f7b7a, 0x745e66cd,  /* [ 28.. 31] */
    0x9823b6e0, 0x9ce2ab57, 0x91a18d8e, 0x95609039,  /* [ 32.. 35] */
    0x8b27c03c, 0x8fe6dd8b, 0x82a5fb52, 0x8664e6e5,  /* [ 36.. 39] */
    0xbe2b5b58, 0xbaea46ef, 0xb7a96036, 0xb3687d81,  /* [ 40.. 43] */
    0xad2f2d84, 0xa9ee3033, 0xa4ad16ea, 0xa06c0b5d,  /* [ 44.. 47] */
    0xd4326d90, 0xd0f37027, 0xddb056fe, 0xd9714b49,  /* [ 48.. 51] */
    0xc7361b4c, 0xc3f706fb, 0xceb42022, 0xca753d95,  /* [ 52.. 55] */
    0xf23a8028, 0xf6fb9d9f, 0xfbb8bb46, 0xff79a6f1,  /* [ 56.. 59] */
    0xe13ef6f4, 0xe5ffeb43, 0xe8bccd9a, 0xec7dd02d,  /* [ 60.. 63] */
    0x34867077, 0x30476dc0, 0x3d044b19, 0x39c556ae,  /* [ 64.. 67] */
    0x278206ab, 0x23431b1c, 0x2e003dc5, 0x2ac12072,  /* [ 68.. 71] */
    0x128e9dcf, 0x164f8078, 0x1b0ca6a1, 0x1fcdbb16,  /* [ 72.. 75] */
    0x018aeb13, 0x054bf6a4, 0x0808d07d, 0x0cc9cdca,  /* [ 76.. 79] */
    0x7897ab07, 0x7c56b6b0, 0x71159069, 0x75d48dde,  /* [ 80.. 83] */
    0x6b93dddb, 0x6f52c06c, 0x6211e6b5, 0x66d0fb02,  /* [ 84.. 87] */
    0x5e9f46bf, 0x5a5e5b08, 0x571d7dd1, 0x53dc6066,  /* [ 88.. 91] */
    0x4d9b3063, 0x495a2dd4, 0x44190b0d, 0x40d816ba,  /* [ 92.. 95] */
    0xaca5c697, 0xa864db20, 0xa527fdf9, 0xa1e6e04e,  /* [ 96.. 99] */
    0xbfa1b04b, 0xbb60adfc, 0xb6238b25, 0xb2e29692,  /* [100..103] */
    0x8aad2b2f, 0x8e6c3698, 0x832f1041, 0x87ee0df6,  /* [104..107] */
    0x99a95df3, 0x9d684044, 0x902b669d, 0x94ea7b2a,  /* [108..111] */
    0xe0b41de7, 0xe4750050, 0xe9362689, 0xedf73b3e,  /* [112..115] */
    0xf3b06b3b, 0xf771768c, 0xfa325055, 0xfef34de2,  /* [116..119] */
    0xc6bcf05f, 0xc27dede8, 0xcf3ecb31, 0xcbffd686,  /* [120..123] */
    0xd5b88683, 0xd1799b34, 0xdc3abded, 0xd8fba05a,  /* [124..127] */
    0x690ce0ee, 0x6dcdfd59, 0x608edb80, 0x644fc637,  /* [128..131] */
    0x7a089632, 0x7ec98b85, 0x738aad5c, 0x774bb0eb,  /* [132..135] */
    0x4f040d56, 0x4bc510e1, 0x46863638, 0x42472b8f,  /* [136..139] */
    0x5c007b8a, 0x58c1663d, 0x558240e4, 0x51435d53,  /* [140..143] */
    0x251d3b9e, 0x21dc2629, 0x2c9f00f0, 0x285e1d47,  /* [144..147] */
    0x36194d42, 0x32d850f5, 0x3f9b762c, 0x3b5a6b9b,  /* [148..151] */
    0x0315d626, 0x07d4cb91, 0x0a97ed48, 0x0e56f0ff,  /* [152..155] */
    0x1011a0fa, 0x14d0bd4d, 0x19939b94, 0x1d528623,  /* [156..159] */
    0xf12f560e, 0xf5ee4bb9, 0xf8ad6d60, 0xfc6c70d7,  /* [160..163] */
    0xe22b20d2, 0xe6ea3d65, 0xeba91bbc, 0xef68060b,  /* [164..167] */
    0xd727bbb6, 0xd3e6a601, 0xdea580d8, 0xda649d6f,  /* [168..171] */
    0xc423cd6a, 0xc0e2d0dd, 0xcda1f604, 0xc960ebb3,  /* [172..175] */
    0xbd3e8d7e, 0xb9ff90c9, 0xb4bcb610, 0xb07daba7,  /* [176..179] */
    0xae3afba2, 0xaafbe615, 0xa7b8c0cc, 0xa379dd7b,  /* [180..183] */
    0x9b3660c6, 0x9ff77d71, 0x92b45ba8, 0x9675461f,  /* [184..187] */
    0x8832161a, 0x8cf30bad, 0x81b02d74, 0x857130c3,  /* [188..191] */
    0x5d8a9099, 0x594b8d2e, 0x5408abf7, 0x50c9b640,  /* [192..195] */
    0x4e8ee645, 0x4a4ffbf2, 0x470cdd2b, 0x43cdc09c,  /* [196..199] */
    0x7b827d21, 0x7f436096, 0x7200464f, 0x76c15bf8,  /* [200..203] */
    0x68860bfd, 0x6c47164a, 0x61043093, 0x65c52d24,  /* [204..207] */
    0x119b4be9, 0x155a565e, 0x18197087, 0x1cd86d30,  /* [208..211] */
    0x029f3d35, 0x065e2082, 0x0b1d065b, 0x0fdc1bec,  /* [212..215] */
    0x3793a651, 0x3352bbe6, 0x3e119d3f, 0x3ad08088,  /* [216..219] */
    0x2497d08d, 0x2056cd3a, 0x2d15ebe3, 0x29d4f654,  /* [220..223] */
    0xc5a92679, 0xc1683bce, 0xcc2b1d17, 0xc8ea00a0,  /* [224..227] */
    0xd6ad50a5, 0xd26c4d12, 0xdf2f6bcb, 0xdbee767c,  /* [228..231] */
    0xe3a1cbc1, 0xe760d676, 0xea23f0af, 0xeee2ed18,  /* [232..235] */
    0xf0a5bd1d, 0xf464a0aa, 0xf9278673, 0xfde69bc4,  /* [236..239] */
    0x89b8fd09, 0x8d79e0be, 0x803ac667, 0x84fbdbd0,  /* [240..243] */
    0x9abc8bd5, 0x9e7d9662, 0x933eb0bb, 0x97ffad0c,  /* [244..247] */
    0xafb010b1, 0xab710d06, 0xa6322bdf, 0xa2f33668,  /* [248..251] */
    0xbcb4666d, 0xb8757bda, 0xb5365d03, 0xb1f740b4,  /* [252..255] */
};

typedef struct{	GPIO_TypeDef 		*gpio_x;
								uint16_t				gpio_pin;
}Gpio_PortPin_t;

typedef struct{	TIM_HandleTypeDef		*tim_handle;
								uint32_t						tim_channel;
								uint16_t						tim_isStarted;
								uint16_t						last_led_tim_val;
}Timer_Led_t;

typedef struct{	uint16_t						result_dn;
								uint16_t						gaugeMinScale;
								ADC_HandleTypeDef		*gauge_handle;
}Lightness_Gauge_t;

typedef struct{
								SPI_HandleTypeDef		*spi_handle;
								Gpio_PortPin_t			*spi_mcs_gpio;
}Spi_Channel_t;




typedef struct{	uint32_t	error_LightW;
								uint32_t	error_LightIR;
								uint16_t	lightnessWDN;
								uint16_t	lightnessIRDN;
}VCP_Light_Reply_t;

typedef struct{	uint32_t	error;
								uint16_t	lightnessDN;
								uint16_t	unused;
}VCP_SetLed_Reply_t;

typedef struct SensorStatus_s
{
	uint32_t SOFTSTATE;
	uint32_t SOFTSTATE_EPSTATE;
	uint32_t SOFTSTATE_FPSTATE;
	uint32_t SOFTSTATE_SSLSTATE;
	uint32_t EP2FLUSH;//Unused in Nassau
	uint32_t JUSTWOKE;
	uint32_t STATECHANGED;
	uint32_t EP2INSIZE;//Unused in Nassau
	uint32_t EP2INDONE;//Unused in Nassau
	uint32_t RUNNING;
	uint32_t EP1OUT;
	uint32_t EP1IN;
	uint32_t DRDY;
	uint32_t JUSTRESET;
	uint32_t ALIVE;
	uint32_t EP1INSIZE;//Only used in Nassau
	uint32_t SOFTSTATE2;//Only used in Nassau
}Sensor_Status_t;

typedef unsigned long long ep0status_t;

//command blob for Execute FW command.
typedef struct command_blob_s
{
	//uint8_t endpoint;				//end point
	uint8_t name;					//command name
	//uint8_t *pCommandStruct;		//command structure
	//uint32_t commandStructLength;	//command structure lenght
	uint8_t *pData;					//data buffer
	uint32_t dataLength;			//data buffer size
}command_blob_t;

typedef enum{ OFF_REPLYSENT, CMDWAIT, CMDPROC, REPLY }epstate_status;

Timer_Led_t led_W		= {	.tim_handle				= &htim10,
												.tim_channel			= TIM_CHANNEL_1,
												.tim_isStarted			= 0,
												.last_led_tim_val	= 0};

Timer_Led_t led_IR	= {	.tim_handle				= &htim11,
												.tim_channel			= TIM_CHANNEL_1,
												.tim_isStarted			= 0,
												.last_led_tim_val	= 0 };

Lightness_Gauge_t lightnessGauge		=	{	.gauge_handle		= &hadc3,
																				.result_dn			= 0x0000,
																				.gaugeMinScale	= 0x0C };

VCP_Light_Reply_t vcp_lightdn_reply	=	{	.error_LightW		= 0xffffffff,
																				.error_LightIR	= 0xffffffff,
																				.lightnessWDN		= 0xffff,
																				.lightnessIRDN	= 0xffff };

VCP_SetLed_Reply_t vcp_setLed_reply	= {	.error					= 0xffffffff,
																				.lightnessDN		= 0xffff,
																				.unused					= 0xffff};

Gpio_PortPin_t led_R = {.gpio_x		= GPIOG,
												.gpio_pin	= GPIO_PIN_2};

Gpio_PortPin_t led_G = {.gpio_x		= GPIOG,
												.gpio_pin	= GPIO_PIN_3};

Gpio_PortPin_t led_B = {.gpio_x		= GPIOG,
												.gpio_pin	= GPIO_PIN_4};

Gpio_PortPin_t	mcs_spi5 ={	.gpio_x		= SPI5_CS_GPIO_Port,
														.gpio_pin = SPI5_CS_Pin};

Gpio_PortPin_t	mcs_spi1 ={	.gpio_x		= SPI1_CS_GPIO_Port,
														.gpio_pin = SPI1_CS_Pin};														

Spi_Channel_t		spi_channel_module = {.spi_handle		= &hspi5,
																			.spi_mcs_gpio	= &mcs_spi5};

Spi_Channel_t		spi_channel_extend = {.spi_handle		= &hspi1,
																			.spi_mcs_gpio	= &mcs_spi1};

Sensor_Status_t Sensor_Status;

vcsfw_reply_get_version_t version;

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
void assert_mcs(Gpio_PortPin_t *spi_mcs_gpio, GPIO_PinState state);

uint32_t FpTidleSet(uint16_t idletime, uint32_t timeout);
uint32_t FpGetVersion(uint8_t *arrVersion, uint32_t size, uint32_t timeout);
uint32_t getStatus(Sensor_Status_t *oSensorStatus);
uint32_t executeCmd(uint8_t cmdname, uint8_t *cmdbufp, uint32_t buflen, uint8_t *replybufp, uint32_t replybuflen, bool crc, uint16_t *replystatus, uint32_t timeout);
uint32_t executeCmdExt(uint8_t cmdname, uint8_t *cmdbufp, uint32_t buflen, uint8_t *replybufp, uint32_t replybuflen, bool crc, uint16_t *replystatus, uint32_t *replySize, uint32_t timeout);
uint32_t writeCmd(command_blob_t cmd, bool crc, uint32_t timeout);
uint32_t readCmd(uint8_t *arrRep, uint32_t size, bool crc, uint32_t timeout);
uint32_t spiWriteRead(Spi_Channel_t *spiChannel, uint8_t *cmdBuf, uint32_t cmdLen, uint8_t * rplBuf, uint32_t rplLen, uint32_t timeout);

uint32_t spiWrite(Spi_Channel_t *spiChannel, uint8_t *cmdBuf, uint32_t cmdLen, uint32_t timeout);
uint32_t spiRead(Spi_Channel_t *spiChannel, uint8_t *rplBuf, uint32_t rplLen, uint32_t timeout);
uint32_t crc32_calc(const uint8_t *datap, unsigned int nbytes, uint32_t crc);
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
  MX_ADC3_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_I2C2_Init();
  MX_SPI5_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
	iEnableSPIVCC( SPIVCC_1V80 );
	iEnableSENSORVCC( SENSOR_VCC_3V30 ) ;

	HAL_Delay(200);

//	FpGetVersion((uint8_t*)&version, sizeof(vcsfw_reply_get_version_t), 200);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
		VCP_CMD_process( );
		HAL_Delay(5);
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
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  HAL_GPIO_WritePin(SPI5_CS_GPIO_Port, SPI5_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SPIVCC_200mV_Pin|SPIVCC_100mV_Pin|SPIVCC_50mV_Pin|SPIVCC_800mV_Pin 
                          |SPIVCC_400mV_Pin|SPIVCC_1600mV_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

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

  /*Configure GPIO pin : SPI5_CS_Pin */
  GPIO_InitStruct.Pin = SPI5_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI5_CS_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

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
	lightnessGauge->result_dn = 0xffff;
	HAL_ADC_Start(lightnessGauge->gauge_handle);
	HAL_ADC_PollForConversion(lightnessGauge->gauge_handle, 2000);
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
					vcp_lightdn_reply.error_LightW	= 0xffffffff,
					vcp_lightdn_reply.error_LightIR	= 0xffffffff,
					vcp_lightdn_reply.lightnessWDN	= 0xffff;
					vcp_lightdn_reply.lightnessIRDN	= 0xffff;
					if(s_RxBuff.UserRxBufferFS[2] >= 0x10 || s_RxBuff.UserRxBufferFS[4] >= 0x10)
					{
						if(s_RxBuff.UserRxBufferFS[2] >= 0x10)
						{
							vcp_lightdn_reply.error_LightW = ERROR_SET_DN_EXCEEDED;
						}
						if(s_RxBuff.UserRxBufferFS[4] >= 0x10)
						{
							vcp_lightdn_reply.error_LightIR = ERROR_SET_DN_EXCEEDED;
						}
						CDC_Transmit_FS((uint8_t *)(&vcp_lightdn_reply), sizeof(vcp_lightdn_reply));      //send back the detected light adc value
						break;
					}

					if(s_RxBuff.UserRxBufferFS[4] >= 0x10)
					{
						vcp_lightdn_reply.error_LightIR = ERROR_SET_DN_EXCEEDED;
						CDC_Transmit_FS((uint8_t *)(&vcp_lightdn_reply), sizeof(vcp_lightdn_reply));      //send back the detected light adc value
						break;
					}
					uint16_t lightnessW_target = (uint16_t)s_RxBuff.UserRxBufferFS[1] + (((uint16_t)s_RxBuff.UserRxBufferFS[2])<<8);
					uint16_t lightnessIR_target = (uint16_t)s_RxBuff.UserRxBufferFS[3] + (((uint16_t)s_RxBuff.UserRxBufferFS[4])<<8);

					//adjust IR Led to lightnessIR_target
					vTurnOffLed(&led_W);
					HAL_Delay(200);
//				vTurnOffLed(&led_IR);
					if (AutoAdjustLed(&led_IR, &lightnessGauge, lightnessIR_target) == -2)
					{
						vcp_lightdn_reply.error_LightIR = ERROR_ADJUST_TO_TARGET_FAIL;
					}
					else
					{
						vcp_lightdn_reply.error_LightIR = lightnessGauge.gauge_handle->ErrorCode;
					}
					vcp_lightdn_reply.lightnessIRDN = lightnessGauge.result_dn;

					//adjust W Led to lightness_target
//				vTurnOffLed(&led_W);
					vTurnOffLed(&led_IR);
					HAL_Delay(200);
					if (AutoAdjustLed(&led_W, &lightnessGauge, lightnessW_target) == -2)
					{
						vcp_lightdn_reply.error_LightW = ERROR_ADJUST_TO_TARGET_FAIL;
					}
					else
					{
						vcp_lightdn_reply.error_LightW = lightnessGauge.gauge_handle->ErrorCode;
					}
					vcp_lightdn_reply.lightnessWDN = lightnessGauge.result_dn;
					vTurnOnLed(&led_W, led_W.last_led_tim_val);		//re-light on IR LED again
					vTurnOnLed(&led_IR, led_IR.last_led_tim_val);		//re-light on IR LED again

					CDC_Transmit_FS((uint8_t *)(&vcp_lightdn_reply), sizeof(vcp_lightdn_reply));      //send back the detected light adc value
					break;
				}


				case VCP_CMD_SET_LED_TIM:						//process cmd set led lightness by timer number, and return light digital number
				{
					if(s_RxBuff.UserRxBufferFS[2] >= 0x10 || s_RxBuff.UserRxBufferFS[4] >= 0x10)
					{
						vcp_setLed_reply.error = ERROR_SET_DN_EXCEEDED;
						CDC_Transmit_FS((uint8_t *)(&vcp_lightdn_reply), sizeof(vcp_lightdn_reply));      //send back the detected light adc value
						break;
					}
					vTurnOnLed(&led_W, (uint16_t)s_RxBuff.UserRxBufferFS[1] + (((uint16_t)s_RxBuff.UserRxBufferFS[2])<<8));
					vTurnOnLed(&led_IR, (uint16_t)s_RxBuff.UserRxBufferFS[3] + (((uint16_t)s_RxBuff.UserRxBufferFS[4])<<8));
					HAL_Delay(200);
					GetLightness(&lightnessGauge);

					vcp_setLed_reply.error = lightnessGauge.gauge_handle->ErrorCode;
					vcp_setLed_reply.lightnessDN = lightnessGauge.result_dn;
					CDC_Transmit_FS( (uint8_t *)(&vcp_setLed_reply), sizeof(VCP_SetLed_Reply_t));       //send back the detected light adc value
					break;
				}


				case VCP_CMD_GET_LED_DN:			// process cmd get led digtal number back , and return light digital number
				{
					GetLightness(&lightnessGauge);
					vcp_setLed_reply.error = lightnessGauge.gauge_handle->ErrorCode;
					vcp_setLed_reply.lightnessDN = lightnessGauge.result_dn;
					CDC_Transmit_FS( (uint8_t *)(&vcp_setLed_reply), sizeof(VCP_SetLed_Reply_t));       //send back the detected light adc value
					break;
				}

				case VCP_CMD_FP_GETVER:
				{
//					uint32_t rc = 0;
					FpGetVersion((uint8_t*)&version, sizeof(vcsfw_reply_get_version_t), 200);
					CDC_Transmit_FS( (uint8_t *)(&version), sizeof(version));
//						CDC_Transmit_FS( (uint8_t *)(&version.buildnum), sizeof(version.buildnum));
//						CDC_Transmit_FS( (uint8_t *)(&version.buildtime), sizeof(version.buildtime));
//						CDC_Transmit_FS( (uint8_t *)(&version.device_type), sizeof(version.device_type));
//						CDC_Transmit_FS( (uint8_t *)(&version.formalrel), sizeof(version.formalrel));
//						CDC_Transmit_FS( (uint8_t *)(&version.iface), sizeof(version.iface));
//						CDC_Transmit_FS( (uint8_t *)(&version.otpsig), sizeof(version.otpsig));
//						CDC_Transmit_FS( (uint8_t *)(&version.otpspare1), sizeof(version.otpspare1));
//						CDC_Transmit_FS( (uint8_t *)(&version.patch), sizeof(version.patch));
//						CDC_Transmit_FS( (uint8_t *)(&version.patchsig), sizeof(version.patchsig));
//						CDC_Transmit_FS( (uint8_t *)(&version.platform), sizeof(version.platform));
//						CDC_Transmit_FS( (uint8_t *)(&version.product), sizeof(version.product));
//						CDC_Transmit_FS( (uint8_t *)(&version.reserved), sizeof(version.reserved));
//						CDC_Transmit_FS( (uint8_t *)(&version.security), sizeof(version.security));
//						CDC_Transmit_FS( (uint8_t *)(&version.serial_number), sizeof(version.serial_number));
//						CDC_Transmit_FS( (uint8_t *)(&version.siliconrev), sizeof(version.siliconrev));
//						CDC_Transmit_FS( (uint8_t *)(&version.target), sizeof(version.target));
//						CDC_Transmit_FS( (uint8_t *)(&version.vmajor), sizeof(version.vmajor));
//						CDC_Transmit_FS( (uint8_t *)(&version.vminor), sizeof(version.vminor));

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
		HAL_Delay(200);
		GetLightness(lightGauge);
		if (abs(lightGauge->result_dn - dn_target) > lightGauge->gaugeMinScale)
		{
			return -2;
		}
		else
		{
			return 0;
		}
	}
	uint16_t led_low = LED_MIN, led_high = LED_MAX;
	uint8_t count = 0;
	while (led_low <= led_high)
	{
		count++;
		if(count >= LIGHT_AUTOADJUST_TIME_MAX)
		{
			return -2;
		}
		uint16_t led_middle = (led_low + led_high) / 2;
		vTurnOnLed(ledTimer, led_middle);
		HAL_Delay(200);
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

void assert_mcs(Gpio_PortPin_t *spi_mcs_gpio, GPIO_PinState state)
{
	HAL_GPIO_WritePin(spi_mcs_gpio->gpio_x, spi_mcs_gpio->gpio_pin, state);
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
	ledTimer->tim_isStarted = 1;
//	HAL_Delay(200);
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
	ledTimer->tim_isStarted = 0;
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
		if(led_W.tim_isStarted != 0)
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
		if(led_IR.tim_isStarted != 0)
		{
			vTurnOffLed(&led_IR);
		}
    else
		{
			vTurnOnLed(&led_IR, 0x0fff);
		}
//		CDC_Transmit_FS( UserTxBuffer,  sizeof(UserTxBuffer) );
  }
	return;
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

uint32_t FpTidleSet(uint16_t idletime, uint32_t timeout)
{
    //Logging::GetLogger()->Log("FpModule FpTidleSet() idletime=%u", idletime);
    uint32_t rc = 0;
    uint8_t arrTidleSet[2] = { 0, 0 };//vcsfw_cmd_tidle_set_t
    memcpy(arrTidleSet, (uint8_t*)&idletime, sizeof(uint16_t));
    vcsfw_generic_reply_t ReplyStatus;
    rc = executeCmd(VCSFW_CMD_TIDLE_SET, arrTidleSet, 2, NULL, NULL, true, &ReplyStatus.status, timeout);
    if (0 != rc || 0 != ReplyStatus.status)
    {
        return 0 != rc ? rc : ReplyStatus.status;
    }

    return rc;
}

uint32_t FpGetVersion(uint8_t *arrVersion, uint32_t size, uint32_t timeout)
{
    //Logging::GetLogger()->Log("FpModule FpGetVersion()");
    uint32_t rc = 0;

    //check params
    if (NULL == arrVersion)
    {
        return ERROR_PARAMETER;
    }

    vcsfw_generic_reply_t ReplyStatus;
    rc = executeCmd(VCSFW_CMD_GET_VERSION, NULL, NULL, arrVersion, size, true, &ReplyStatus.status, timeout);
    if (0 != rc || 0 != ReplyStatus.status)
    {
        return 0 != rc ? rc : ReplyStatus.status;
    }

    return rc;
}

//Sensor_Status_t oSensorStatus;
uint32_t getStatus(Sensor_Status_t *oSensorStatus)
{
//		deassertmcs();
    uint32_t status = 0;
		ep0status_t ep0val = 0;
		uint32_t ep0size = EP0SIZE_BIG;
		uint8_t  cmdbuf[2] = { EPSELBYTE_INTEGRIFY(EPSELBYTE_EP0IN), 0x00 };   /* 0 is dummy byte */
		status = spiWriteRead(&spi_channel_module, cmdbuf, sizeof(cmdbuf), (uint8_t *)&ep0val, ep0size, 200);
    if (0 == status)
    {
        //parse
        oSensorStatus->SOFTSTATE = EXTRACT32(ep0val, EP0IN_SOFTSTATE, EP0IN_SOFTSTATE_B);
        oSensorStatus->SOFTSTATE_EPSTATE = EXTRACT32(ep0val, EP0IN_SOFTSTATE_EP1STATE, EP0IN_SOFTSTATE_EP1STATE_B);
        oSensorStatus->SOFTSTATE_FPSTATE = EXTRACT32(ep0val, EP0IN_SOFTSTATE_FPSTATE, EP0IN_SOFTSTATE_FPSTATE_B);
        oSensorStatus->SOFTSTATE_SSLSTATE = EXTRACT32(ep0val, EP0IN_SOFTSTATE_SSLSTATE, EP0IN_SOFTSTATE_SSLSTATE_B);
        oSensorStatus->EP2FLUSH = EXTRACT32(ep0val, EP0IN_EP2FLUSH, EP0IN_EP2FLUSH_B);
        oSensorStatus->JUSTWOKE = EXTRACT32(ep0val, EP0IN_JUSTWOKE, EP0IN_JUSTWOKE_B);
        oSensorStatus->STATECHANGED = EXTRACT32(ep0val, EP0IN_STATECHANGED, EP0IN_STATECHANGED_B);
        oSensorStatus->EP2INSIZE = EXTRACT32(ep0val, EP0IN_EP2INSIZE, EP0IN_EP2INSIZE_B);
        oSensorStatus->EP2INDONE = EXTRACT32(ep0val, EP0IN_EP2INDONE, EP0IN_EP2INDONE_B);
        oSensorStatus->RUNNING = EXTRACT32(ep0val, EP0IN_RUNNING, EP0IN_RUNNING_B);
        oSensorStatus->EP1OUT = EXTRACT32(ep0val, EP0IN_EP1OUT, EP0IN_EP1OUT_B);
        oSensorStatus->EP1IN = EXTRACT32(ep0val, EP0IN_EP1IN, EP0IN_EP1IN_B);
        oSensorStatus->DRDY = EXTRACT32(ep0val, EP0IN_DRDY, EP0IN_DRDY_B);
        oSensorStatus->JUSTRESET = EXTRACT32(ep0val, EP0IN_JUSTRESET, EP0IN_JUSTRESET_B);
        oSensorStatus->ALIVE = EXTRACT32(ep0val, EP0IN_ALIVE, EP0IN_ALIVE_B);
        oSensorStatus->EP1INSIZE = EXTRACT32(ep0val, EP0IN_EP1INSIZE, EP0IN_EP1INSIZE_B);
        oSensorStatus->SOFTSTATE2 = EXTRACT32(ep0val, EP0IN_SOFTSTATE2, EP0IN_SOFTSTATE2_B);
    }

    return status;
}

uint32_t executeCmd(uint8_t cmdname, uint8_t *cmdbufp, uint32_t buflen, uint8_t *replybufp, uint32_t replybuflen, bool crc, uint16_t *replystatus, uint32_t timeout)
{
    uint32_t replySize;
    return executeCmdExt(cmdname, cmdbufp, buflen, replybufp, replybuflen, crc, replystatus, &replySize, timeout);
}

uint32_t executeCmdExt(uint8_t cmdname, uint8_t *cmdbufp, uint32_t buflen, uint8_t *replybufp, uint32_t replybuflen, bool crc, uint16_t *replystatus, uint32_t *replySize, uint32_t timeout)
{
    uint32_t rc = 0;
    uint32_t timeoutVal = timeout;

    command_blob_t cmd;
    cmd.name = cmdname;
    cmd.pData = cmdbufp;
    cmd.dataLength = buflen;

    //get status. To see if the module if ready to execute command.
 //   Sensor_Status_t Sensor_Status;
    do{
        rc = getStatus(&Sensor_Status);
        if (0 != rc)
            return rc;
        timeoutVal--;

        if (1 == Sensor_Status.EP1OUT && (CMDWAIT == Sensor_Status.SOFTSTATE_EPSTATE || OFF_REPLYSENT == Sensor_Status.SOFTSTATE_EPSTATE))
        {
            rc = 0;
            break;
        }
    } while (0 != timeoutVal);

    if (0 == timeoutVal)
        return ERROR_TIME_OUT;

    //write command
    rc = writeCmd(cmd, crc, timeout);
    if (0 != rc)
        return rc;

    timeoutVal = timeout;
    do{
        rc = getStatus(&Sensor_Status);
        if (0 != rc)
            return rc;
        timeoutVal--;

        if (1 == Sensor_Status.EP1IN && REPLY == Sensor_Status.SOFTSTATE_EPSTATE && 1 == Sensor_Status.DRDY)
        {
            rc = 0;
            break;
        }
    } while (0 != timeoutVal);

    if (0 == timeoutVal)
        return ERROR_TIME_OUT;

    //read command
    if (0 != rc)
        return rc;

    uint32_t sizeRead = Sensor_Status.EP1INSIZE;
    uint8_t arrRead[sizeRead];
    memset(arrRead, 0, sizeRead);
    rc = readCmd(arrRead, sizeRead, crc, timeout);
    if (0 == rc)
    {
        if (replybuflen >= sizeRead)
        {
            //reduce status bytes
            *replySize = sizeRead - sizeof(uint16_t);
            if (replySize > 0)
            {
                memcpy(replybufp, &(arrRead[sizeof(uint16_t)]), *replySize);
            }
        }
        else
        {
            if (0 != replybuflen && NULL != replybufp)
            {
                memcpy(replybufp, &(arrRead[sizeof(uint16_t)]), replybuflen);
                *replySize = replybuflen;
            }
        }

        //copy status bytes
        memcpy(&replystatus, arrRead, sizeof(uint16_t));
    }
    free(arrRead);
    return rc;
}


uint32_t writeCmd(command_blob_t cmd, bool crc, uint32_t timeout)
{
    //write CMD to EP1OUT 0xA2
    uint32_t rc = 0;

    //fill cmd bufp
    uint32_t fullcmdlength = 0;

    //allocate command buff length = endpoint + command_name + command_data_lenght + crc
    fullcmdlength = sizeof(uint8_t) + sizeof(uint8_t) + cmd.dataLength + (crc ? sizeof(uint32_t) : 0);
    uint8_t fullcmdbufp[fullcmdlength];
    memset(fullcmdbufp, 0, fullcmdlength);

    fullcmdbufp[0] = EPSELBYTE_INTEGRIFY(EPSELBYTE_EP1OUT);     //endpoint 1
    fullcmdbufp[1] = cmd.name;

    if ((NULL != cmd.pData) && (0 != cmd.dataLength))
    {
        memcpy(fullcmdbufp + 2 * sizeof(uint8_t), cmd.pData, cmd.dataLength);
    }
    if (crc)
    {
        //calc crc without endpoint, only with command name and data.
        uint32_t ncrc = crc32_calc(&fullcmdbufp[1], cmd.dataLength + sizeof(uint8_t), ~0UL);
        HOSTTOBIG32(fullcmdbufp + fullcmdlength - sizeof(uint32_t), ncrc);
    }

    //send cmd
//    rc = _pFpBridge->Write(fullcmdbufp, fullcmdlength, true);
		rc = spiWrite(&spi_channel_module, fullcmdbufp, fullcmdlength, 200);
    free(fullcmdbufp);

    return rc;
}


uint32_t readCmd(uint8_t *arrRep, uint32_t size, bool crc, uint32_t timeout)
{
	//read return value from EP1IN 0x23
	uint32_t rc=0;

	uint8_t  epSel = EPSELBYTE_INTEGRIFY(EPSELBYTE_EP1IN);
	uint8_t arrEPSel[2] = { epSel, 0xFF };
//    rc = _pFpBridge->Write(&(arrEPSel[0]), 2, false);
	if (0 != rc)
			return rc;

	// Read reply from the sensor
	uint32_t replyLength = size + (crc ? sizeof(uint32_t) : 0);
	uint8_t arrReplyBuf[replyLength];
	memset(arrReplyBuf, 0, replyLength);
//    rc = _pFpBridge->Read(arrReplyBuf, replyLength, true);
	rc = spiWriteRead(&spi_channel_module, &(arrEPSel[0]), 2, arrReplyBuf, replyLength, 200);
	if (0 == rc)
	{
		//crc
		if (crc)
		{
			uint32_t crcVal = 0;
			memcpy(&crcVal, &(arrReplyBuf[replyLength - sizeof(uint32_t)]), sizeof(uint32_t));

			// Check the CRC-32 on the received data and make sure it's good.
			uint32_t localcrc = 0;
			/*
			 * big ep0
			 * If we're not using the big EP0 then the length is part of the CRC calculation
			 */
			uint32_t ep1inlen = size;
			//localcrc = crc32_calc((const uint8_t *)&ep1inlen, sizeof(uint32_t), ~0UL);
			localcrc = crc32_calc(arrReplyBuf, ep1inlen, ~0UL);
			//wprintf(L"REPLY_READCRC: over %u bytes read CRC 0x%08lx, expecting 0x%08lx\n", ep1inlen, BIGTOHOST32(&crcVal), localcrc);
			if (BIGTOHOST32(&crcVal) != localcrc)
			{
					return ERROR_CRC_VERIFY;
			}

            memcpy(arrRep, arrReplyBuf, replyLength - sizeof(uint32_t));
		}
		else
		{
				memcpy(arrRep, arrReplyBuf, replyLength);
		}
  }

	free(arrReplyBuf);
	return rc;
}

uint32_t spiWriteRead(Spi_Channel_t *spiChannel, uint8_t *cmdBuf, uint32_t cmdLen, uint8_t *rpl, uint32_t rplLen, uint32_t timeout)
{
	uint32_t status = 0;
	uint8_t rplBuf[rplLen];
	uint8_t dummy_cmd[rplLen];
	memset(dummy_cmd, 0x00, rplLen);
	assert_mcs(spiChannel->spi_mcs_gpio, GPIO_PIN_RESET);
//	HAL_Delay(10);
	status = HAL_SPI_Transmit(spiChannel->spi_handle, (uint8_t *)&cmdBuf[0], cmdLen, timeout);
	status = HAL_SPI_TransmitReceive(spiChannel->spi_handle, dummy_cmd, (uint8_t *)&rplBuf, rplLen, timeout);
	memcpy(rpl, rplBuf, rplLen);
	assert_mcs(spiChannel->spi_mcs_gpio, GPIO_PIN_SET);
	return status;
}

uint32_t spiWrite(Spi_Channel_t *spiChannel, uint8_t *cmdBuf, uint32_t cmdLen, uint32_t timeout)
{
	assert_mcs(spiChannel->spi_mcs_gpio, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit(spiChannel->spi_handle, cmdBuf, cmdLen, timeout) != HAL_OK)
	{
		return 1;
	}
	assert_mcs(spiChannel->spi_mcs_gpio, GPIO_PIN_SET);
	return 0;
}

uint32_t spiRead(Spi_Channel_t *spiChannel, uint8_t *rplBuf, uint32_t rplLen, uint32_t timeout)
{
	assert_mcs(spiChannel->spi_mcs_gpio, GPIO_PIN_RESET);
	if(HAL_SPI_Receive(spiChannel->spi_handle, (uint8_t *)&rplBuf, rplLen, timeout) != HAL_OK)
	{
		return 1;
	}
	assert_mcs(spiChannel->spi_mcs_gpio, GPIO_PIN_SET);
	return 0;
}



uint32_t crc32_calc(const uint8_t *datap, unsigned int nbytes, uint32_t crc)
{
#if 1
    while (nbytes != 0) {
        crc = crc32_table[((*datap++ << 24) ^ crc) >> 24] ^ (crc << 8);
        nbytes--;
    }

    return ~crc;
#else
#define CRC32_POLY  0x04c11db7
    unsigned int        bitnum, highbit;
    uint8_t             val;

    while (nbytes != 0) {
        val = *datap++;
        bitnum = 8;
        while (bitnum != 0) {
            highbit = (crc & 0x80000000) >> 31;
            crc <<= 1;
            if ((highbit ^ ((val >> 7) & 1)) == 1) {
                crc ^= CRC32_POLY;
            }
            val <<= 1;

            bitnum--;
        }

        nbytes--;
    }


    return ~crc;
#endif

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
