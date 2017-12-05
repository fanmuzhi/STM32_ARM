
/* Includes ------------------------------------------------------------------*/
#include "peripheral_devices.h"

#define LED_MIN 									0x0000
#define LED_MAX 									0x0fff
#define DN_LED_MIN 								0x0000
#define DN_LED_MAX 								0x0fff
#define TIME_LED_DELAY	 					100			//	ms, time after waiting the LED intensity stabal after light on
#define LIGHT_AUTOADJUST_TIME_MAX	20
													
extern ADC_HandleTypeDef hadc3;

extern I2C_HandleTypeDef hi2c2;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi5;

extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;													


Gpio_PortPin_t led_R = {.gpio_x		= GPIOG,
												.gpio_pin	= GPIO_PIN_2};

Gpio_PortPin_t led_G = {.gpio_x		= GPIOG,
												.gpio_pin	= GPIO_PIN_3};

Gpio_PortPin_t led_B = {.gpio_x		= GPIOG,
												.gpio_pin	= GPIO_PIN_4};


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
																		

Gpio_PortPin_t	mcs_spi5 ={	.gpio_x		= SPI5_CS_GPIO_Port,
														.gpio_pin = SPI5_CS_Pin};

Gpio_PortPin_t	mcs_spi1 ={	.gpio_x		= SPI1_CS_GPIO_Port,
														.gpio_pin = SPI1_CS_Pin};														

Spi_Channel_t		spi_channel_module = {.spi_handle		= &hspi5,
																			.spi_mcs_gpio	= &mcs_spi5};

Spi_Channel_t		spi_channel_extend = {.spi_handle		= &hspi1,
																			.spi_mcs_gpio	= &mcs_spi1};		

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

uint16_t GetLightness( Lightness_Gauge_t *lightnessGauge )
{
	lightnessGauge->result_dn = 0xffff;
	HAL_ADC_Start(lightnessGauge->gauge_handle);
	HAL_ADC_PollForConversion(lightnessGauge->gauge_handle, 2000);
	lightnessGauge->result_dn = HAL_ADC_GetValue(lightnessGauge->gauge_handle);
	return 0;
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

void assert_mcs(Gpio_PortPin_t *spi_mcs_gpio, GPIO_PinState state)
{
	HAL_GPIO_WritePin(spi_mcs_gpio->gpio_x, spi_mcs_gpio->gpio_pin, state);
}

uint32_t spiWriteRead(Spi_Channel_t *spiChannel, uint8_t *cmdBuf, uint32_t cmdLen, uint8_t *rpl, uint32_t rplLen, uint32_t timeout)
{
	uint32_t status = 0;
	uint8_t rplBuf[rplLen];
//	uint8_t dummy_cmd[rplLen];
//	memset(dummy_cmd, 0x00, rplLen);
	assert_mcs(spiChannel->spi_mcs_gpio, GPIO_PIN_RESET);
//	HAL_Delay(10);
//	status = HAL_SPI_Transmit(spiChannel->spi_handle, (uint8_t *)&cmdBuf[0], cmdLen, timeout);
	status = HAL_SPI_TransmitReceive(spiChannel->spi_handle, (uint8_t *)&cmdBuf[0], (uint8_t *)&rplBuf, rplLen, timeout);
	memcpy(rpl, rplBuf, rplLen);
	assert_mcs(spiChannel->spi_mcs_gpio, GPIO_PIN_SET);
	return status;
}

uint32_t spiWrite(Spi_Channel_t *spiChannel, uint8_t *cmdBuf, uint32_t cmdLen, bool deassert_mcs_after, uint32_t timeout)
{
	assert_mcs(spiChannel->spi_mcs_gpio, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit(spiChannel->spi_handle, cmdBuf, cmdLen, timeout) != HAL_OK)
	{
		return 1;
	}
	if (deassert_mcs_after == true)
	{
		assert_mcs(spiChannel->spi_mcs_gpio, GPIO_PIN_SET);
	}
	return 0;
}

uint32_t spiRead(Spi_Channel_t *spiChannel, uint8_t *rplBuf, uint32_t rplLen, bool deassert_mcs_after, uint32_t timeout)
{
	assert_mcs(spiChannel->spi_mcs_gpio, GPIO_PIN_RESET);
	if(HAL_SPI_Receive(spiChannel->spi_handle, (uint8_t *)&rplBuf, rplLen, timeout) != HAL_OK)
	{
		return 1;
	}
		if (deassert_mcs_after == true)
	{
		assert_mcs(spiChannel->spi_mcs_gpio, GPIO_PIN_SET);
	}
	return 0;
}
