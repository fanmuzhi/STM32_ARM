/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VCP_COMM_INTERFACE_H
#define __VCP_COMM_INTERFACE_H

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "vcsTypes.h"
#include "vcsfw_v4.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"



#define VCP_CMD_SET_LED_DN				0x10
#define VCP_CMD_SET_LED_TIM				0x11
#define VCP_CMD_GET_LED_DN 				0x20
#define VCP_CMD_FP_GETVER					0x50
#define VCP_CMD_IOTA_FIND					0x60
#define VCP_CMD_GET_IMG						0x70
//#define VCP_CMD_GET_IMG_STELLER		0x75

#define VCP_FPBRIDGE_CMD_OPEN							0x95
#define VCP_FPBRIDGE_CMD_CLOSE						0x90
#define VCP_FPBRIDGE_CMD_GET_IDENTITY			0x0A
#define VCP_FPBRIDGE_CMD_SETVOLTAGES			0x01
#define VCP_FPBRIDGE_CMD_SETVOLTAGEPP3V3	0x02
#define VCP_FPBRIDGE_CMD_SETVOLTAGEPP1V8	0x03
#define VCP_FPBRIDGE_CMD_SET_GPIO					0x30
#define VCP_FPBRIDGE_CMD_CHECK_GPIO				0x40
#define VCP_FPBRIDGE_CMD_GETCURRENTVALUES	0xB0
#define VCP_FPBRIDGE_CMD_WRITE						0xA1
#define VCP_FPBRIDGE_CMD_READ							0xA0



	 
#define ERROR_NONE											0x00000000
#define ERROR_SET_LED_EXCEEDED					0xF0000001
#define ERROR_SET_DN_EXCEEDED						0xF0000002
#define ERROR_ADJUST_TO_TARGET_FAIL			0xF0000003


#define DEFAULT_TIMEOUT_VALUE						2000

typedef struct{	uint32_t	error_LightW;
								uint32_t	error_LightIR;
								uint16_t	lightnessWDN;
								uint16_t	lightnessIRDN;
								uint16_t	ledW_Set;
								uint16_t	ledIR_Set;
}VCP_Light_Reply_t;

typedef struct{	uint32_t	error;
								uint16_t	lightnessDN;
								uint16_t	unused;
}VCP_SetLed_Reply_t;	 

int8_t VCP_CMD_process( void );
	 
#endif
