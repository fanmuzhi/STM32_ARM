
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

typedef struct MeasureINA226
{
	uint8_t ucChannel;
	uint8_t ucRange;
	uint8_t ucCalibration;
	
	uint16_t usConfigRegValue;
	uint16_t usMaskEnableRegValue;
	uint16_t usAlertLimitRegValue;
	int16_t sCurrentRegValue;
	int16_t sShuntVoltageRegValue;
	uint16_t usBusVoltageRegValue;
	uint16_t usPowerRegValue;
	
	float fCurrent;
	float fShuntVoltage;
	float fBusVoltage;
	float fPower;
}MeasureINA226_t;

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
																				.gaugeMinScale	= 0x06 };
																		

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
		HAL_Delay(50);
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
		HAL_Delay(50);
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


uint8_t ucMeasureCurrentS( MeasureINA226_t *INA226_parameter )
{
	uint8_t pValue[2] = {0};
	uint16_t DevAddress, usCalibration_reg;
	HAL_StatusTypeDef IIC_Status;
	
	// power channel selection
	if( INA226_parameter -> ucChannel == SPI_CHANNEL )
	{
		DevAddress = SPIVCC_I2C_ADDRESS;
	}
	else if( INA226_parameter -> ucChannel == MODULE_CHANNEL )
	{
		DevAddress = VCC_I2C_ADDRESS;
	}
	else
	{
		return INA226_ERROR_PARAMETER;
	}
	
	// calibration selection
	if( INA226_parameter -> ucCalibration == NON_CALIBRATION )
	{
		HAL_GPIO_WritePin( CALIBRATE_LOW_GPIO_Port, CALIBRATE_LOW_Pin, GPIO_PIN_RESET);	// calibration disable
	}
	else if( INA226_parameter -> ucCalibration == CALIBRATION )
	{
		// do calibration
		HAL_GPIO_WritePin( CALIBRATE_LOW_GPIO_Port, CALIBRATE_LOW_Pin, GPIO_PIN_SET);	// calibration enable
	}
	else
	{
		return INA226_ERROR_PARAMETER;
	}
	
	// range selection
	if( INA226_parameter -> ucRange == MILI_AMPERE )
	{
		HAL_GPIO_WritePin( RANGE_SEL_GPIO_Port, RANGE_SEL_Pin, GPIO_PIN_RESET);	// mA
	}
	else if( INA226_parameter -> ucRange == MICRO_AMPERE )
	{
		HAL_GPIO_WritePin( RANGE_SEL_GPIO_Port, RANGE_SEL_Pin, GPIO_PIN_SET);	// uA
	}
	else
	{
		return INA226_ERROR_PARAMETER;
	}
			
	// set Alert Limit register
	uint8_t pAlertLimit_reg[3] = { INA226_ALERT_LIMIT_REG, ( INA226_parameter -> usAlertLimitRegValue ) >> 8, INA226_parameter -> usAlertLimitRegValue };
	IIC_Status = HAL_I2C_Master_Transmit( &hi2c2, DevAddress, pAlertLimit_reg, 3, 2000 );
	if( IIC_Status != HAL_OK )
	{
		return IIC_Status;
	}
	IIC_Status = HAL_I2C_Master_Receive( &hi2c2, DevAddress, pValue, 2, 2000 );
	if( IIC_Status != HAL_OK )
	{
		return IIC_Status;
	}
	
	// set Mask/Enable register
	uint8_t pMaskEnable_reg[3] = { INA226_MASKENABLE_REG, ( INA226_parameter -> usMaskEnableRegValue ) >> 8, INA226_parameter -> usMaskEnableRegValue };
	IIC_Status = HAL_I2C_Master_Transmit( &hi2c2, DevAddress, pMaskEnable_reg, 3, 2000 );
	if( IIC_Status != HAL_OK )
	{
		return IIC_Status;
	}
	IIC_Status = HAL_I2C_Master_Receive( &hi2c2, DevAddress, pValue, 2, 2000 );
	if( IIC_Status != HAL_OK )
	{
		return IIC_Status;
	}
	
	//set calibration_register_value
	usCalibration_reg = 0.00512 / CURRENT_LSB / ( INA226_parameter -> ucRange == MICRO_AMPERE ? SHUNT_RESISTOR_MICRO : SHUNT_RESISTOR_MILI );
	//uint8_t pCalibration_reg[3] = { INA226_CALIBRATION_REG, 0, 51 };			// current_LSB = 1uA
	//uint8_t pCalibration_reg[3] = { INA226_CALIBRATION_REG, 0x08, 0x00 }; // MT used 0x0800 and current_LSB = 0.025uA
	uint8_t pCalibration_reg[3] = { INA226_CALIBRATION_REG, usCalibration_reg >> 8, usCalibration_reg };
	IIC_Status = HAL_I2C_Master_Transmit( &hi2c2, DevAddress, pCalibration_reg, 3, 2000 );
	if( IIC_Status != HAL_OK )
	{
		return IIC_Status;
	}
	IIC_Status = HAL_I2C_Master_Receive( &hi2c2, DevAddress, pValue, 2, 2000 );
	if( IIC_Status != HAL_OK )
	{
		return IIC_Status;
	}
		
	// config/init INA226, write the config register, will trig the convertion for trigged mode
	uint8_t pConfig_reg[3] = { INA226_CONFIG_REG, ( INA226_parameter -> usConfigRegValue ) >> 8, INA226_parameter -> usConfigRegValue };
	IIC_Status = HAL_I2C_Master_Transmit( &hi2c2, DevAddress, pConfig_reg, 3, 2000 );
	if( IIC_Status != HAL_OK )
	{
		return IIC_Status;
	}
	IIC_Status = HAL_I2C_Master_Receive( &hi2c2, DevAddress, pValue, 2, 2000 );
	if( IIC_Status != HAL_OK )
	{
		return IIC_Status;
	}
	if( pValue[0] != INA226_parameter ->usConfigRegValue >> 8 && pValue[1] != INA226_parameter ->usConfigRegValue )
	{
		return INA226_CONFIG_SET_FAIL;
	}
	
	// read Mask/Enable register, check convert finish
	uint8_t regAddress = INA226_MASKENABLE_REG;
	IIC_Status = HAL_I2C_Master_Transmit( &hi2c2, DevAddress, &regAddress, 1, 2000 );
	if( IIC_Status != HAL_OK )
	{
		return IIC_Status;
	}
	do
	{
		IIC_Status = HAL_I2C_Master_Receive( &hi2c2, DevAddress, pValue, 2, 2000 );
		if( IIC_Status != HAL_OK )
		{
			return IIC_Status;
		}
		HAL_Delay(10);	// why MUST delay??? otherwise will timeout when repeat call HAL_I2C_Master_Receive() around 1000 times.
	}while( ( pValue[1] & INA226_CVRF ) == 0 );
	INA226_parameter -> usMaskEnableRegValue = (( pValue[0] << 8 ) & 0xFF00 ) +  pValue[1];
	
	// read current register
	regAddress = INA226_CURRENT_REG;
	IIC_Status = HAL_I2C_Master_Transmit( &hi2c2, DevAddress, &regAddress, 1, 2000 );
	if( IIC_Status != HAL_OK )
	{
		return IIC_Status;
	}
	IIC_Status = HAL_I2C_Master_Receive( &hi2c2, DevAddress, pValue, 2, 2000 );
	if( IIC_Status != HAL_OK )
	{
		return IIC_Status;
	}
	INA226_parameter -> sCurrentRegValue = (( pValue[0] << 8 ) & 0xFF00 ) +  pValue[1];
	INA226_parameter -> fCurrent = INA226_parameter -> sCurrentRegValue * CURRENT_LSB;
	
	// read shunt voltage
	regAddress = INA226_SHUNT_VOLTAGE_REG;
	IIC_Status = HAL_I2C_Master_Transmit( &hi2c2, DevAddress, &regAddress, 1, 2000 );
	if( IIC_Status != HAL_OK )
	{
		return IIC_Status;
	}
	IIC_Status = HAL_I2C_Master_Receive( &hi2c2, DevAddress, pValue, 2, 2000 );
	if( IIC_Status != HAL_OK )
	{
		return IIC_Status;
	}
	INA226_parameter -> sShuntVoltageRegValue = (( pValue[0] << 8 ) & 0xFF00 ) +  pValue[1];
	INA226_parameter -> fShuntVoltage = INA226_parameter -> sShuntVoltageRegValue * 2.5 / 1000000;				// LSB = 2.5uV
	
	// read BUS voltage
	regAddress = INA226_BUS_VOLTAGE_REG;
	IIC_Status = HAL_I2C_Master_Transmit( &hi2c2, DevAddress, &regAddress, 1, 2000 );
	if( IIC_Status != HAL_OK )
	{
		return IIC_Status;
	}
	IIC_Status = HAL_I2C_Master_Receive( &hi2c2, DevAddress, pValue, 2, 2000 );
	if( IIC_Status != HAL_OK )
	{
		return IIC_Status;
	}
	INA226_parameter -> usBusVoltageRegValue = (( pValue[0] << 8 ) & 0xFF00 ) +  pValue[1];
	INA226_parameter -> fBusVoltage = INA226_parameter -> usBusVoltageRegValue * 1.25 / 1000;							// LSB = 1.25mV
	
	// read power
	regAddress = INA226_POWER_REG;
	IIC_Status = HAL_I2C_Master_Transmit( &hi2c2, DevAddress, &regAddress, 1, 2000 );
	if( IIC_Status != HAL_OK )
	{
		return IIC_Status;
	}
	IIC_Status = HAL_I2C_Master_Receive( &hi2c2, DevAddress, pValue, 2, 2000 );
	if( IIC_Status != HAL_OK )
	{
		return IIC_Status;
	}
	INA226_parameter -> usPowerRegValue = (( pValue[0] << 8 ) & 0xFF00 ) +  pValue[1];
	INA226_parameter -> fPower = INA226_parameter -> usPowerRegValue * 25 * CURRENT_LSB;									// LSB = 25 * CURRENT_LSB
	
	// set the range back to mA
	HAL_GPIO_WritePin( RANGE_SEL_GPIO_Port, RANGE_SEL_Pin, GPIO_PIN_RESET);	// mA
	
	return 0;
}

void assert_mcs(Gpio_PortPin_t *spi_mcs_gpio, GPIO_PinState state)
{
	HAL_GPIO_WritePin(spi_mcs_gpio->gpio_x, spi_mcs_gpio->gpio_pin, state);
}

uint32_t spiWriteRead(Spi_Channel_t *spiChannel, uint8_t *cmdBuf, uint8_t *rpl, uint32_t Len, uint32_t timeout)
{
	uint32_t status = 0;
//	uint8_t rplBuf[0x3800];
	uint8_t *rplBuf = NULL;
	rplBuf = (uint8_t *) malloc(Len);
	
	assert_mcs(spiChannel->spi_mcs_gpio, GPIO_PIN_RESET);
//	HAL_Delay(50);
//	status = HAL_SPI_Transmit(spiChannel->spi_handle, (uint8_t *)&cmdBuf[0], cmdLen, timeout);
	status = HAL_SPI_TransmitReceive(spiChannel->spi_handle, cmdBuf, rplBuf, Len, timeout);
	assert_mcs(spiChannel->spi_mcs_gpio, GPIO_PIN_SET);
	memcpy(rpl, rplBuf, Len);
	free(rplBuf);
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
