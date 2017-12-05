/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PERIPHERAL_DEVICES_H
#define __PERIPHERAL_DEVICES_H

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <string.h>
#include "stm32f7xx_hal.h"

typedef enum { false, true }bool;

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


extern ADC_HandleTypeDef hadc3;

extern I2C_HandleTypeDef hi2c2;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi5;

extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;

	 
	 
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
	 
void vTurnOnRgbLed( uint16_t r, uint16_t g, uint16_t b );
void vTurnOffRgbLed(void);

uint16_t vTurnOnLed( Timer_Led_t *ledTimer, uint16_t ucLightness );
uint16_t vTurnOffLed( Timer_Led_t *ledTimer);
uint16_t GetLightness( Lightness_Gauge_t *lightnessGauge );
int32_t AutoAdjustLed(Timer_Led_t *ledTimer, Lightness_Gauge_t *lightGauge, uint16_t dn_target);

int iEnableSPIVCC( int iVoltage );
int iEnableSENSORVCC( int iVoltage );
void assert_mcs(Gpio_PortPin_t *spi_mcs_gpio, GPIO_PinState state);	 
uint32_t spiWriteRead(Spi_Channel_t *spiChannel, uint8_t *cmdBuf, uint32_t cmdLen, uint8_t * rplBuf, uint32_t rplLen, uint32_t timeout);
//uint32_t spiWrite(Spi_Channel_t *spiChannel, uint8_t *cmdBuf, uint32_t cmdLen, uint32_t timeout);
uint32_t spiWrite(Spi_Channel_t *spiChannel, uint8_t *cmdBuf, uint32_t cmdLen, bool deassert_mcs_after, uint32_t timeout);
//uint32_t spiRead(Spi_Channel_t *spiChannel, uint8_t *rplBuf, uint32_t rplLen, uint32_t timeout);
uint32_t spiRead(Spi_Channel_t *spiChannel, uint8_t *rplBuf, uint32_t rplLen, bool deassert_mcs_after, uint32_t timeout);

#endif
