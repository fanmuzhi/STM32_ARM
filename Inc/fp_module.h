/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FP_MODULE_H
#define __FP_MODULE_H

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <string.h>

#include "vcsTypes.h"
#include "vcsfw_v4.h"
#include "peripheral_devices.h"
	 
#define TIMEOUT_VALUE 2000

//error code
#define ERROR_TIME_OUT					0x1500
#define ERROR_CRC_VERIFY				0x1501
#define ERROR_BL_MODE					0x1502
#define ERROR_PARAM_UNDEFINE			0x1503
#define ERROR_TYPE						0x1504
#define ERROR_PARAMETER					0x1505
#define ERROR_RESULT_NOTFOUND			0x1506
#define ERROR_REPLY_LENGTH_TOO_SHORT	0x1507
#define ERROR_NOT_IMPLEMENTED			0x1508

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

typedef enum { MISSION_FW, IOTA, FIB }FlashReadType;
typedef enum { BIGGEST, SHA256 }FlashReadFlags;
//typedef enum FlashEraseType{ MISSION_FW, IOTA, MF_IOTA, FIB, SDB, ALL }flashEraseType;

typedef enum  { FW_BL = 1, LNA_BL, DM, CDM, REG32BLK, REG16BLK, DIMS, ACQOPT, XSREG8BLK, STRIDEBLK, STATS, ACQCFG, ORIENTPITCH, IMGPROC_CROP, IMGPROC_DYNBPDET, IMGPROC_3X3SEPSQ, IMGPROC_DISABLE, EXTFPS_OTP_TEMPCORRECT, EXTFPS_OTP_DIEIMPRINT, EXTFPS_SIGCTRL, EXTFPS_SUNDRYREGS }FrameTagType;

typedef enum { Baseline, Signal }WOFCalibrateType;


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

typedef struct sensor_current_s
{
	uint32_t vccAvgCurrent;
	uint32_t spivccAvgCurrent;
	uint32_t vccPeekCurrent;
	uint32_t spivccPeekCurrent;
}sensor_current_t;

typedef struct ext_register_info_s
{
	uint32_t address;
	uint8_t  value;
}ext_register_info_t;

#define SET_EXTREGINFO(p, addr, val)  { \
    (p)->address = addr;    \
    (p)->value = val;   \
}

//uint32_t FpSteller_FpGetImage(uint8_t *arrImage, uint32_t size, uint8_t *arrParameter, uint32_t parameterSize, uint32_t nframes, uint32_t timeout);

uint32_t FpCalibrate(uint32_t timeout);

uint32_t FpFrameStateGet(FrameTagType type, uint8_t *pRespData, uint32_t *oRespSize, uint32_t nTimeout);

uint32_t FpGetImage(uint8_t *arrImage, uint32_t size, uint8_t *arrParameter, uint32_t parameterSize, uint32_t nframes, uint32_t timeout);

uint32_t FpIotafind(uint8_t * arrIotaData,  uint32_t *fullSize, uint32_t IotaType, uint32_t timeout);

uint32_t FpIotawrite(uint8_t *arrIotadata, uint32_t size, uint16_t type, uint32_t timeout);

uint32_t FpPeek(uint32_t address, uint8_t opsize, uint32_t *ovalue, uint32_t timeout);

uint32_t FpPoke(uint32_t address, uint32_t value, uint8_t opsize, uint32_t timeout);

uint32_t FpTestRun(uint8_t patchcmd, uint8_t *arrResponse, uint32_t size, uint32_t timeout);

uint32_t FpTestRun2(uint8_t patchCmd, uint8_t *arrResponse, uint32_t size, uint32_t timeout);

uint32_t FpPowerOn(uint32_t vcc, uint32_t spivcc, uint32_t timeout);

uint32_t FpPowerOff(uint32_t timeout);

uint32_t FpTidleSet(uint16_t idletime, uint32_t timeout);

uint32_t FpGetVersion(uint8_t *arrVersion, uint32_t size, uint32_t timeout);

uint32_t getStatus(Sensor_Status_t *oSensorStatus);

uint32_t executeCmd(uint8_t cmdname, uint8_t *cmdbufp, uint32_t buflen, uint8_t *replybufp, uint32_t replybuflen, bool crc, uint16_t *replystatus, uint32_t *replySize, uint32_t timeout);

uint32_t writeCmd(command_blob_t cmd, bool crc, uint32_t timeout);

uint32_t readCmd(uint8_t *arrRep, uint32_t size, bool crc, uint32_t timeout);

uint32_t crc32_calc(const uint8_t *datap, unsigned int nbytes, uint32_t crc);

#endif
