/* Includes ------------------------------------------------------------------*/
#include "fp_module.h"

#include "vcsTypes.h"
#include "vcsfw_v4.h"

/*
 * A table of precomputed CRC values.
 *  This table was generated using the script 'crc32.pl' that's
 *  checked into engineering/private/impl/micro/falcon/patch/moduletest/crc32.pl
 * This table was computed using a polynomial of 0x[1]04c11b7
 *  (which reversed is 0xedb88320.[8]).  This
 *  is the standard CRC-32 polynomial used with HDLC, Ethernet, etc.
 *  So it's got to be good, right?
 * Note that this used the 'big endian' flag -e.
 */
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


//EPSTATE status
typedef enum{ OFF_REPLYSENT, CMDWAIT, CMDPROC, REPLY }epstate_status;

//FPSTATE status 
typedef enum{ UNKNOWN, ABSENT, STILL, MOVING }fpstate_status;

/* Given a 32-bit value, swap the bytes */
#define ENDIANSWAP32(val)                                                   \
    ((((val)& 0xff) << 24)                                                 \
    | ((((val) >> 8) & 0xff) << 16)                                        \
    | ((((val) >> 16) & 0xff) << 8)                                        \
    | (((val) >> 24) & 0xff))

/* Convert a known big-endian value to host (little) endian value */
#define BIGTOHOST32(valp)                                                   \
    ENDIANSWAP32(*((const uint32_t *)(valp)))

/* Convert a local 32-bit value to a big endian value */
#define HOSTTOBIG32(destp, val) (*((uint32_t *) (destp)) = ENDIANSWAP32(val))

/*
 * Endian switchers.  We assume that dactyl is running on a little-endian
 *  machine.
 */
#define SENSORTOHOST8(valp)     (*((const uint8_t *) (valp)))
#define SENSORTOHOST16(valp)    (*((const uint16_t *) (valp)))
#define SENSORTOHOST32(valp)    (*((const uint32_t *) (valp)))

#define HOSTTOSENSOR8(destp, val) (*((uint8_t *) (destp)) = (val))
#define HOSTTOSENSOR16(destp, val) (*((uint16_t *) (destp)) = (val))
#define HOSTTOSENSOR32(destp, val) (*((uint32_t *) (destp)) = (val))

#define NELEM(x)        (sizeof((x)) / sizeof((x)[0]))

/*
 * Mask a 64-bit value and produce a 32-bit one.
 */
#define EXTRACT32(val, mask, shift)                                         \
    (unsigned int)(((val)& (mask)) >> (shift))

/* Given an EPSELBYTE, set the high bits accordingly */
#define EPSELBYTE_INTEGRIFY(x)      \
    (((x)& ~0xe0) | (((~(x)& 1) << 7) | ((~(x)& 2) << 5) | ((~(x)& 4) << 3)))

/*
 * The end points supported on SPI interface
 */
#define EPSELBYTE_EP0IN     ((0 << 1) | 1)
#define EPSELBYTE_EP1OUT    ((1 << 1) | 0)
#define EPSELBYTE_EP1IN     ((1 << 1) | 1)
#define EPSELBYTE_EP2IN     ((2 << 1) | 1)
#define EPSELBYTE_EP3OUT    ((3 << 1) | 0)
#define EPSELBYTE_RESET     0xf0
#define EPSELBYTE_ENTERSLEEP        0xE8
#define EPSELBYTE_EXITSLEEP         0x17

#define EP0SIZE                    4
#define EP0SIZE_BIG                8

/*
 * Fields in the EP0IN register.
 */
#define EP0IN_SOFTSTATE           0x0000001f
#define EP0IN_SOFTSTATE_B             0
#define EP0IN_SOFTSTATE_N             5
#define EP0IN_SOFTSTATE_EP1STATE  0x00000003
#define EP0IN_SOFTSTATE_EP1STATE_B    0
#define EP0IN_SOFTSTATE_EP1STATE_N    2
#define EP0IN_SOFTSTATE_EP1STATE_OFF        0
#define EP0IN_SOFTSTATE_EP1STATE_REPLYSENT  0
#define EP0IN_SOFTSTATE_EP1STATE_CMDWAIT    1
#define EP0IN_SOFTSTATE_EP1STATE_CMDPROC    2
#define EP0IN_SOFTSTATE_EP1STATE_REPLY      3
#define EP0IN_SOFTSTATE_FPSTATE   0x0000000c
#define EP0IN_SOFTSTATE_FPSTATE_B     2
#define EP0IN_SOFTSTATE_FPSTATE_N     2
#define EP0IN_SOFTSTATE_FPSTATE_UNKNOWN     0
#define EP0IN_SOFTSTATE_FPSTATE_ABSENT      1
#define EP0IN_SOFTSTATE_FPSTATE_STILL       2
#define EP0IN_SOFTSTATE_FPSTATE_MOVING      3
#define EP0IN_SOFTSTATE_SSLSTATE  0x00000010
#define EP0IN_SOFTSTATE_SSLSTATE_B    4
#define EP0IN_SOFTSTATE_SSLSTATE_N    1
#define EP0IN_EP2FLUSH            0x00000020
#define EP0IN_EP2FLUSH_B              5
#define EP0IN_JUSTWOKE            0x00000040
#define EP0IN_JUSTWOKE_B              6
#define EP0IN_STATECHANGED        0x00000080
#define EP0IN_STATECHANGED_B          7
#define EP0IN_EP2INSIZE           0x01ffff00
#define EP0IN_EP2INSIZE_B             8
#define EP0IN_EP2INSIZE_N             17
#define EP0IN_EP2INDONE           0x02000000
#define EP0IN_EP2INDONE_B             25
#define EP0IN_RUNNING             0x04000000
#define EP0IN_RUNNING_B               26
#define EP0IN_EP1OUT              0x08000000
#define EP0IN_EP1OUT_B                27
#define EP0IN_EP1IN               0x10000000
#define EP0IN_EP1IN_B                 28
#define EP0IN_DRDY                0x20000000
#define EP0IN_DRDY_B                  29
#define EP0IN_JUSTRESET           0x40000000
#define EP0IN_JUSTRESET_B             30
#define EP0IN_ALIVE               0x80000000
#define EP0IN_ALIVE_B                 31

/*
 * The high bits of EP0IN (EP0_IN_SPI_I2C_STS_1)
 */
#define EP0IN_SOFTSTATE2          0xffff000000000000ULL
#define EP0IN_SOFTSTATE2_B        48
#define EP0IN_SOFTSTATE2_N        16
#define EP0IN_EP1INSIZE           0x0000ffff00000000ULL
#define EP0IN_EP1INSIZE_B         32
#define EP0IN_EP1INSIZE_N         16

typedef unsigned long long ep0status_t;

// Loop count for measuring CPCurrent
#define FPMODULE_CPCURRENT_MEASURE_CNT         7

extern Spi_Channel_t spi_channel_module;
extern Spi_Channel_t spi_channel_extend;


// DEFINES IN FP_STELLER
/*
 * Endian switchers.  We assume that dactyl is running on a little-endian
 *  machine.
 */
#define SENSORTOHOST8(valp)     (*((const uint8_t *) (valp)))
#define SENSORTOHOST16(valp)    (*((const uint16_t *) (valp)))
#define SENSORTOHOST32(valp)    (*((const uint32_t *) (valp)))

#define HOSTTOSENSOR8(destp, val) (*((uint8_t *) (destp)) = (val))
#define HOSTTOSENSOR16(destp, val) (*((uint16_t *) (destp)) = (val))
#define HOSTTOSENSOR32(destp, val) (*((uint32_t *) (destp)) = (val))

#define SENSOR_CMDWAIT  1
#define SENSOR_REPLY	3

#define IOTA_IMAGE_DATA_ALIGNMENT_STELLER      4
#define IOTA_WRITE_DATA_SIZE_MAX_STELLER       48000

#define STATUS_ACQNUM_MAX 7


uint32_t FpCalibrate(uint32_t timeout)
{
    //Logging::GetLogger()->Log("FpModule FpCalibrate()");

    uint32_t rc = 0;
	vcsfw_generic_reply_t ReplyStatus = { 0 };
	uint32_t replysize = 0;

    vcsfw_cmd_frame_acq_t frame_acq;
    frame_acq.nframes = 0;
    frame_acq.flags = VCSFW_CMD_FRAME_ACQ_FLAGS_CAL_BL;
    rc = executeCmd(VCSFW_CMD_FRAME_ACQ, (uint8_t*)&frame_acq, sizeof(vcsfw_cmd_frame_acq_t), NULL, 0, true, &ReplyStatus.status, &replysize, timeout);
    if (0 != rc || 0 != ReplyStatus.status)
    {
        return 0 != rc ? rc : ReplyStatus.status;
    }

    return rc;
}

/*
    FpFrameStateGet
    
    Parameters : 
        type        [in]   - FrameTagType
        pRespData   [out]  - response data
        oRespSize   [out]  - response data size
        nTimeout    [in]   - time out period
*/
uint32_t FpFrameStateGet(FrameTagType type, uint8_t *pRespData, uint32_t *oRespSize, uint32_t nTimeout)
{
    //Logging::GetLogger()->Log("FpModule FpFrameStateGet()");

    uint32_t        rc = 0;
    //unsigned int    nFlags = 0x00;
    unsigned int    nFullSize = NULL;
    unsigned int    nReplySize = NULL;

    vcsfw_reply_frame_state_get_t   frameReply;
    vcsfw_cmd_frame_state_get_t     frameStateCmd;
    vcsfw_generic_reply_t           genReply;
	uint32_t						replysize = 0;

    //check params
    if (NULL == pRespData)
    {
        return ERROR_PARAMETER;
    }

    memset(&frameStateCmd, 0, sizeof(frameStateCmd));
    frameStateCmd.tag = (uint8_t)type;

    /*
     * get the full length of the frame tag.
     */
    rc = executeCmd(VCSFW_CMD_FRAME_STATE_GET, (uint8_t*)&frameStateCmd, sizeof(frameStateCmd), (uint8_t*)&frameReply, sizeof(frameReply), true, &genReply.status, &replysize, nTimeout);
    if (0 != rc || 0 != genReply.status)
    {
        return 0 != rc ? rc : genReply.status;
    }

    /*
     * Now we have the full state length.  Allocate a buffer
     *  for it and also a buffer to hold the command reply to fetch
     *  it (possibly in pieces).
     */
    nFullSize = frameReply.fullsize;
    if (0 == nFullSize)
    {
        return ERROR_REPLY_LENGTH_TOO_SHORT;
    }

    /*
     * Now repeatedly issue the command to get whole data of the frame tag
     */
    memset(&frameStateCmd, 0, sizeof(frameStateCmd));
    frameStateCmd.flags = VCSFW_CMD_FRAME_STATE_GET_FLAG_READMAX;
    frameStateCmd.tag = (uint8_t)type;

    /*  
     * ReplySize = sizeof(vcsfw_reply_frame_state_get_t) + sizeof(vcsfw_frame_tag_t) + sizeof(data) + sizeof(vcsfw_frame_tag_t) + sizeof(data) .....
     * 
     */
    nReplySize = nFullSize + sizeof(frameReply);
    uint8_t pReplyBuf[nReplySize];
    rc = executeCmd(VCSFW_CMD_FRAME_STATE_GET, (uint8_t*)&frameStateCmd, sizeof(frameStateCmd), pReplyBuf, nReplySize, true, &genReply.status, &replysize, nTimeout);
    if (0 != rc || 0 != genReply.status)
    {
				free(pReplyBuf);
        return ((rc) ? (rc) : (genReply.status));
    }

    memcpy(&frameReply, pReplyBuf, sizeof(vcsfw_cmd_frame_state_get_t));
    if (nFullSize > frameReply.fullsize) //should equal
    {
        return ERROR_REPLY_LENGTH_TOO_SHORT;
    }

    for (uint32_t i = 0; i < frameReply.fullsize; )
    {
        //parse frametag here
        vcsfw_frame_tag_t frameTag;
        memcpy(&frameTag, &pReplyBuf[i + sizeof(vcsfw_cmd_frame_state_get_t)], sizeof(vcsfw_frame_tag_t));
        uint32_t frameTagSize = frameTag.nwords * 4;   // convert word to bytes

        //find the tag in full return
        if (frameTag.tagid == (uint8_t)type)
        {
            memcpy(pRespData, &pReplyBuf[i + sizeof(vcsfw_cmd_frame_state_get_t)+ sizeof(vcsfw_frame_tag_t)], frameTagSize);
            *oRespSize = frameTagSize;
            //TODO: how to save multiple frame tags
        }
        i += frameTagSize + sizeof(vcsfw_frame_tag_t);
    }
		free(pReplyBuf);
    return rc;
}


uint32_t FpGetImage(uint8_t *arrImage, uint32_t size, uint8_t *arrParameter, uint32_t parameterSize, uint32_t nframes, uint32_t timeout)
{
    //Logging::GetLogger()->Log("FpModule FpGetImage()");

    uint32_t rc = 0;
		vcsfw_generic_reply_t ReplyStatus = { 0 };
		uint32_t replysize = 0;

    //check params
    if (NULL == arrImage)
    {
        return ERROR_PARAMETER;
    }

    //frame acq
    vcsfw_cmd_frame_acq_t frame_acq;
    frame_acq.nframes = 1;//frames
    frame_acq.flags = 0;

		uint8_t *arrFrameAcq = NULL;
		if (NULL == arrParameter || 0 == parameterSize)
		{
			uint8_t arrFrameAcq[sizeof(vcsfw_cmd_frame_acq_t)];
			memcpy(arrFrameAcq, &frame_acq, sizeof(vcsfw_cmd_frame_acq_t));
		}
		else
		{
			uint8_t arrFrameAcq[parameterSize + sizeof(vcsfw_cmd_frame_acq_t)];
			memcpy(arrFrameAcq, &frame_acq, sizeof(vcsfw_cmd_frame_acq_t));
			memcpy(&(arrFrameAcq[sizeof(vcsfw_cmd_frame_acq_t)]), arrParameter, parameterSize);
		}

		rc = executeCmd(VCSFW_CMD_FRAME_ACQ, arrFrameAcq, sizeof(vcsfw_cmd_frame_acq_t)+parameterSize, NULL, 0, true, &ReplyStatus.status, &replysize, timeout);
		free(arrFrameAcq);
    if (0 != rc || 0 != ReplyStatus.status)
    {
        return 0 != rc ? rc : ReplyStatus.status;
    }

    /*
        This is from Matt:
        There must be some other reason DRDY is asserted before the frame is ready.  
        Can you check the acqnum before issuing the FRAME_READ?  
        The FRAME_ACQ command should set acqnum to 0, and when a frame is available, acqnum will be 1.
     */
    uint32_t timeoutVal = 50000;
    Sensor_Status_t Sensor_Status;
    do{
        rc = getStatus(&Sensor_Status);
        if (0 != rc)
            return rc;
        timeoutVal--;
        if (0 != (Sensor_Status.SOFTSTATE2&0x7))
        {
            rc = 0;
            break;
        }
    } while (0 != timeoutVal);
    if (0 == timeoutVal)
        return ERROR_TIME_OUT;

    //test to read frame only
    //frame read
    vcsfw_cmd_frame_read_t frame_read;
    frame_read.nbytes = 0xFFFF;
    frame_read.xfernum = 0;
    frame_read.flags = 0;
    frame_read.offset = 0;
    unsigned int replybuflen = size + sizeof(vcsfw_reply_frame_read_t);
    uint8_t *replybufp = (uint8_t *) malloc(replybuflen);
		
    rc = executeCmd(VCSFW_CMD_FRAME_READ, (uint8_t*)&frame_read, sizeof(vcsfw_reply_frame_read_t), replybufp, replybuflen, true, &ReplyStatus.status, &replysize, timeout);
    
		if (0 != rc || 0 != ReplyStatus.status)
    {
				free(replybufp);
        return 0 != rc ? rc : ReplyStatus.status;
    }

    vcsfw_reply_frame_read_t reply_frame_read;
    memcpy(&reply_frame_read, &(replybufp[0]), sizeof(vcsfw_reply_frame_read_t));

    memcpy(arrImage, &(replybufp[sizeof(vcsfw_reply_frame_read_t)]), size);
		free(replybufp);
		

    //frame finish
    rc = executeCmd(VCSFW_CMD_FRAME_FINISH, NULL, 0, NULL, 0, true, &ReplyStatus.status, &replysize, timeout);
    if (0 != rc || 0 != ReplyStatus.status)
    {
        return 0 != rc ? rc : ReplyStatus.status;
    }

    return rc;
}

/*
	FpIotafind, to execute the IOTA find command.
    
    Parameters : 
		arrIotaData [out]  - Pointer to the data array, exclude the vcsfw_reply_iota_find_t
        fullSize    [out]  - response data size
        IotaType    [in]   - IOTA_ITYPE, 0 = search all 
        timeout     [in]   - time out period
*/
uint32_t FpIotafind(uint8_t *arrIotaData,  uint32_t *fullSize, uint32_t IotaType, uint32_t timeout)
{
    uint32_t rc = 0;
		vcsfw_generic_reply_t ReplyStatus = { 0 };
		uint32_t replysize = 0;

    vcsfw_cmd_iota_find_t iota_find;
    iota_find.itype = IotaType;
    iota_find.flags = VCSFW_CMD_IOTA_FIND_FLAGS_READMAX;
    iota_find.maxniotas = 0;
    iota_find.firstidx = 0;
    iota_find.dummy[0] = 0;
    iota_find.dummy[1] = 0;
    iota_find.offset = 0;
    iota_find.nbytes = 0;

	if (IotaType == 0)	//find all IOTA
	{
		iota_find.flags |= VCSFW_CMD_IOTA_FIND_FLAGS_ALLIOTAS;
	}

	//execute VCSFW_CMD_IOTA_FIND to find full size the iota.
	vcsfw_reply_iota_find_t iota_find_reply_length;
	rc = executeCmd(VCSFW_CMD_IOTA_FIND, (uint8_t*)&iota_find, sizeof(vcsfw_cmd_iota_find_t), (uint8_t*)&iota_find_reply_length, sizeof(vcsfw_reply_iota_find_t), true, &ReplyStatus.status, &replysize, timeout);
	if (0 != rc || 0 != ReplyStatus.status)
	{
		return 0 != rc ? rc : ReplyStatus.status;
	}
	*fullSize = iota_find_reply_length.fullsize;

	//realloc the receive buffer size if needed.
	//if parameter arrIotaData is not NULL, may have memory leakage issue.
	//uint8_t arrIotaData[fullSize];
//	uint8_t *arrIotaData = (uint8_t *)malloc(*fullSize);
	uint32_t offset = 0;
	//int32_t stateleft = fullSize - sizeof(vcsfw_reply_iota_find_t);    //include header : vcsfw_reply_iota_find_t
	int32_t stateleft = *fullSize;

	while (stateleft > 0)
	{
		iota_find.offset = offset;

		uint32_t sizeRead = 0;
		sizeRead = *fullSize;	//try to read max
		uint8_t arrRead[sizeRead];
		memset(arrRead, 0, sizeRead);

		uint32_t replySize = 0;	//actual reply size, exclude status 2 bytes
		rc = executeCmd(VCSFW_CMD_IOTA_FIND, (uint8_t*)&iota_find, sizeof(vcsfw_cmd_iota_find_t), arrRead, sizeRead, true, &ReplyStatus.status, &replySize, timeout);

		if (0 == rc)
		{
			uint32_t piecelen = 0;
			piecelen = replySize - sizeof(vcsfw_reply_iota_find_t);//minus status and reply_iota_find

			//copy data except the vcsfw_reply_iota_find_t header
			memcpy(&(arrIotaData[offset]), &(arrRead[sizeof(vcsfw_reply_iota_find_t)]), piecelen);

			offset += piecelen;
			stateleft -= piecelen;
		}
		free(arrRead);

		if (0 != rc || 0 != ReplyStatus.status)
			return 0 != rc ? rc : ReplyStatus.status;
	}

    return rc;
}

uint32_t FpIotawrite(uint8_t *arrIotadata, uint32_t size, uint16_t type, uint32_t timeout)
{
    uint32_t rc = 0;
	vcsfw_generic_reply_t ReplyStatus = { 0 };
	uint32_t replysize = 0;

    //check params
    if (NULL == arrIotadata||0 == size)
    {
        return ERROR_PARAMETER;
    }

    if (0 == type)
    {
        rc = executeCmd(VCSFW_CMD_IOTA_WRITE, arrIotadata, size, NULL, 0, true, &ReplyStatus.status, &replysize, timeout);
    }
    else
    {
        vcsfw_cmd_iota_write_t cmd_iota_write;
        memset(&cmd_iota_write, 0, sizeof(vcsfw_cmd_iota_write_t));
        cmd_iota_write.itype = type;
        uint8_t arrData[size + sizeof(vcsfw_cmd_iota_write_t)];
        memcpy(arrData, &cmd_iota_write, sizeof(vcsfw_cmd_iota_write_t));
        memcpy(&(arrData[sizeof(vcsfw_cmd_iota_write_t)]), arrIotadata, size);
        rc = executeCmd(VCSFW_CMD_IOTA_WRITE, arrData, size + sizeof(vcsfw_cmd_iota_write_t), NULL, 0, true, &ReplyStatus.status, &replysize, timeout);
        free(arrData);
    }

    if (0 != rc || 0 != ReplyStatus.status)
    {
        rc = (0 != rc) ? rc : ReplyStatus.status;
    }

//#if defined(_INTERNAL_BUILD) || defined(_DEBUG)
//    // Get Time info
//	time_t rawtime;
//	struct tm * timeinfo;
//	wchar_t buffer[80];
//	time(&rawtime);
//	timeinfo = localtime(&rawtime);
//	wcsftime(buffer, 80, L"%y-%m-%d-%H-%M-%S", timeinfo);
//	wstring ws(buffer);
//	string logtime(ws.begin(), ws.end());
//	
//	// Get Error Code
//	char strBuf[100];
//	string strRetStatus("");
//	if (0 != rc)
//	{
//		sprintf(strBuf, "_WriteFail_0x%X_", ReplyStatus.status);
//		strRetStatus = strBuf;
//	}
//
//	// Get IOTA Type
//	sprintf(strBuf, "0x%X", type);
//	string strIotaType = strBuf;
//
//	// Write IOTA Dump
//	string 	strFileName("IOTA_Type_" + strIotaType + "_" + logtime + strRetStatus + ".bin");
//	FILE*		fileDump = NULL;
//	if (fopen_s(&fileDump, strFileName.c_str(), "wb") == NULL)
//	{
//		fwrite(arrIotadata, sizeof(vcsUint8_t), size, fileDump);
//		fclose(fileDump);
//	}
//#endif

    return rc;
}

uint32_t FpPeek(uint32_t address, uint8_t opsize, uint32_t *ovalue, uint32_t timeout)
{
    uint32_t rc = 0;
		vcsfw_generic_reply_t ReplyStatus = { 0 };
		uint32_t replysize = 0;

    vcsfw_cmd_peek_t cmd_peek;
    cmd_peek.address = address;
    cmd_peek.opsize = opsize;

    rc = executeCmd(VCSFW_CMD_PEEK, (uint8_t*)&cmd_peek, sizeof(vcsfw_cmd_peek_t), (uint8_t*)&ovalue, sizeof(uint32_t), true, &ReplyStatus.status, &replysize, timeout);
    if (0 != rc || 0 != ReplyStatus.status)
    {
        return 0 != rc ? rc : ReplyStatus.status;
    }

    return rc;
}

uint32_t FpPoke(uint32_t address, uint32_t value, uint8_t opsize, uint32_t timeout)
{
    uint32_t rc = 0;
		vcsfw_generic_reply_t ReplyStatus = { 0 };
		uint32_t replysize = 0;
    vcsfw_cmd_poke_t cmd_poke;
    cmd_poke.address = address;
    cmd_poke.value = value;
    cmd_poke.opsize = opsize;

    rc = executeCmd(VCSFW_CMD_POKE, (uint8_t*)&cmd_poke, sizeof(vcsfw_cmd_poke_t), NULL, 0, true, &ReplyStatus.status, &replysize, timeout);
    if (0 != rc || 0 != ReplyStatus.status)
    {
        return 0 != rc ? rc : ReplyStatus.status;
    }

    return rc;
}


uint32_t FpTestRun(uint8_t patchcmd, uint8_t *arrResponse, uint32_t size, uint32_t timeout)
{
    //Logging::GetLogger()->Log("FpModule FpTestRun()");

    /*
     *  Combined the command VCSFW_CMD_TEST_RUN + Sub command. 
     *  Then get the response.
     */

    uint32_t rc = 0;
		vcsfw_generic_reply_t ReplyStatus = { 0 };
		uint32_t replysize = 0;

    uint8_t arrTestRun[4] = { patchcmd, 0, 0, 0 };
    rc = executeCmd(VCSFW_CMD_TEST_RUN, arrTestRun, 4, arrResponse, size, true, &ReplyStatus.status, &replysize, timeout);
    if (0 != rc || 0 != ReplyStatus.status)
    {
        return 0 != rc ? rc : ReplyStatus.status;
    }

    return rc;
}

uint32_t FpTestRun2(uint8_t patchCmd, uint8_t *arrResponse, uint32_t size, uint32_t timeout)
{
    //Logging::GetLogger()->Log("FpModule FpTestRun2()");

    /*
     *  Send command patchCmd directly. 
     *  Then get the response.
     */

  uint32_t rc = 0;
	vcsfw_generic_reply_t ReplyStatus = { 0 };
	uint32_t replysize = 0;

  rc = executeCmd(patchCmd, NULL, NULL, arrResponse, size, true, &ReplyStatus.status, &replysize, timeout);
  if (0 != rc || 0 != ReplyStatus.status)
  {
        return 0 != rc ? rc : ReplyStatus.status;
  }

  return rc;
}

uint32_t FpPowerOn(uint32_t vcc, uint32_t spivcc, uint32_t timeout)
{
    //Logging::GetLogger()->Log("FpModule PowerOn()");
    uint32_t rc = 0;

		iEnableSPIVCC( spivcc );
		iEnableSENSORVCC( vcc ) ;

    return rc;
}

uint32_t FpPowerOff(uint32_t timeout)
{
    //Logging::GetLogger()->Log("FpModule PowerOff()");
    uint32_t rc = 0;

		iEnableSPIVCC( SPIVCC_0V00 );
		iEnableSENSORVCC( SENSOR_VCC_0V00 ) ;

    return rc;
}

uint32_t FpTidleSet(uint16_t idletime, uint32_t timeout)
{
    //Logging::GetLogger()->Log("FpModule FpTidleSet() idletime=%u", idletime);
    uint32_t rc = 0;
    uint8_t arrTidleSet[2] = { 0, 0 };//vcsfw_cmd_tidle_set_t
    memcpy(arrTidleSet, (uint8_t*)&idletime, sizeof(uint16_t));
    vcsfw_generic_reply_t ReplyStatus;
		uint32_t replysize = 0;
    rc = executeCmd(VCSFW_CMD_TIDLE_SET, arrTidleSet, 2, NULL, NULL, true, &ReplyStatus.status, &replysize, timeout);
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
		uint32_t replysize = 0;
    vcsfw_generic_reply_t ReplyStatus;
    rc = executeCmd(VCSFW_CMD_GET_VERSION, NULL, NULL, arrVersion, size, true, &ReplyStatus.status, &replysize, timeout);
    if (0 != rc || 0 != ReplyStatus.status)
    {
        return 0 != rc ? rc : ReplyStatus.status;
    }
    return rc;
}


uint32_t getStatus(Sensor_Status_t *oSensorStatus)
{
//		deassertmcs();
    uint32_t status = 0;
//		ep0status_t ep0val = 0;
//		uint32_t ep0size = EP0SIZE_BIG;
		uint8_t  cmdbuf[2] = { EPSELBYTE_INTEGRIFY(EPSELBYTE_EP0IN), 0 };   /* 0 is dummy byte */
		status = spiWrite(&spi_channel_module, cmdbuf, sizeof(cmdbuf), false, 2000);
		if (0 != status)
    {
        return status;
    }
		ep0status_t ep0val;
    uint32_t ep0size = EP0SIZE_BIG;
		uint8_t dummy_cmd[ep0size];
		memset(dummy_cmd, 0x00, ep0size);
//		status = spiWrite(&spi_channel_module, dummy_cmd, ep0size, false, 2000);
//		status = spiRead(&spi_channel_module, (uint8_t *)&ep0val, ep0size, true, 2000);
		status = spiWriteRead(&spi_channel_module, dummy_cmd, ep0size, (uint8_t *)&ep0val, ep0size, 2000);
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

uint32_t executeCmd(uint8_t cmdname, uint8_t *cmdbufp, uint32_t buflen, uint8_t *replybufp, uint32_t replybuflen, bool crc, uint16_t *replystatus, uint32_t *replySize, uint32_t timeout)
{

		uint32_t rc = 0;
    uint32_t timeoutVal = timeout;

    command_blob_t cmd;
    cmd.name = cmdname;
    cmd.pData = cmdbufp;
    cmd.dataLength = buflen;

    //get status. To see if the module if ready to execute command.
		Sensor_Status_t Sensor_Status;
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
            if (*replySize > 0)
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
		rc = spiWrite(&spi_channel_module, fullcmdbufp, fullcmdlength, true, 2000);
    free(fullcmdbufp);

    return rc;
}


uint32_t readCmd(uint8_t *arrRep, uint32_t size, bool crc, uint32_t timeout)
{
	//read return value from EP1IN 0x23
	uint32_t rc=0;

	uint8_t  epSel = EPSELBYTE_INTEGRIFY(EPSELBYTE_EP1IN);
	uint8_t arrEPSel[2] = { epSel, 0xFF };
	rc = spiWrite(&spi_channel_module, &(arrEPSel[0]), 2, false, 2000);
//    rc = _pFpBridge->Write(&(arrEPSel[0]), 2, false);
	if (0 != rc)
			return rc;
	
	// Read reply from the sensor
	uint32_t replyLength = size + (crc ? sizeof(uint32_t) : 0);
	uint8_t arrReplyBuf[replyLength];
	memset(arrReplyBuf, 0, replyLength);
	uint8_t dummy_cmd[replyLength];
	memset(dummy_cmd, 0xFF, replyLength);
	rc = spiWriteRead(&spi_channel_module, dummy_cmd, replyLength, arrReplyBuf, replyLength, 2000);
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
