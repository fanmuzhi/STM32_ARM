/* Includes ------------------------------------------------------------------*/
#include "vcp_comm_interface.h"


#include "peripheral_devices.h"
#include "fp_module.h"

extern Timer_Led_t led_W;

extern Timer_Led_t led_IR;

extern Lightness_Gauge_t lightnessGauge;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern s_RxBuff_t s_RxBuff;


VCP_Light_Reply_t vcp_lightdn_reply	=	{	.error_LightW		= 0xffffffff,
																				.error_LightIR	= 0xffffffff,
																				.lightnessWDN		= 0xffff,
																				.lightnessIRDN	= 0xffff };

VCP_SetLed_Reply_t vcp_setLed_reply	= {	.error					= 0xffffffff,
																				.lightnessDN		= 0xffff,
																				.unused					= 0xffff};
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
					vcsfw_reply_get_version_t version;
					FpGetVersion((uint8_t*)&version, sizeof(vcsfw_reply_get_version_t), 2000U);
					CDC_Transmit_FS( (uint8_t *)(&version), sizeof(version));
					break;
				}
				
				case VCP_CMD_IOTA_FIND:
				{
//					uint32_t rc = 0;
					//try find only one IOTA
					uint8_t *arrIotadata = NULL;
					uint32_t replyLength = 0;
					FpIotafind(arrIotadata, &replyLength, 0U, 2000);
					CDC_Transmit_FS( (uint8_t *)(&arrIotadata), replyLength);
					break;
				}
				
				case VCP_CMD_GET_IMG:
				{
					uint32_t rc = 0;
					uint32_t rowNumber = 88;// _dims.frame_nlines;
					uint32_t columnNumber = 80;// _dims.line_npix;
					
					uint8_t *arrImage = NULL;
					arrImage = (uint8_t *) malloc( rowNumber * columnNumber * 2 );
					rc = FpGetImage(arrImage, rowNumber*columnNumber*2, (uint8_t *)0, 0U, 1U, 2000);
					if(0 == rc) 
					{
						rc = CDC_Transmit_FS( (uint8_t *)(&arrImage), rowNumber*columnNumber*2);
					}
					if (0 == rc)
					{
						free(arrImage);
					}
					
					break;
				}

				default:
					break;
			}
			
			s_RxBuff.IsCommandDataReceived = 0;
			
		}
    return 1;
}

