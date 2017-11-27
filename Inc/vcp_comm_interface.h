/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VCP_COMM_INTERFACE_H
#define __VCP_COMM_INTERFACE_H

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

#include "vcsTypes.h"
#include "vcsfw_v4.h"
#include "fp_module.h"

#define VCP_CMD_SET_LED_DN				0x10
#define VCP_CMD_SET_LED_TIM				0x11
#define VCP_CMD_GET_LED_DN 				0x20
#define VCP_CMD_FP_GETVER					0x50

#define ERROR_NONE											0x00000000
#define ERROR_SET_LED_EXCEEDED					0xF0000001
#define ERROR_SET_DN_EXCEEDED						0xF0000002
#define ERROR_ADJUST_TO_TARGET_FAIL			0xF0000003

typedef struct{	uint32_t	error_LightW;
								uint32_t	error_LightIR;
								uint16_t	lightnessWDN;
								uint16_t	lightnessIRDN;
}VCP_Light_Reply_t;

typedef struct{	uint32_t	error;
								uint16_t	lightnessDN;
								uint16_t	unused;
}VCP_SetLed_Reply_t;	 
	 
int8_t VCP_CMD_process( void );
	 
#endif
