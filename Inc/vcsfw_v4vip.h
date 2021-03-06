/* -*- mode: c; tab-width: 4 -*- */
/* $Header$ */

/*
 * Copyright (c) 2010-2015 Synaptics Incorporated.  All rights reserved.
 */

/*
 * vcsfw_v4vip.h - This file defines the command and reply structures
 * of specific Falcon commands required for VeriSign VIP OTP support.
 *
 * NOTE:
 * The interfaces defined here is expected to be implemented only
 * in a patch.
 * 
 * Smbat Tonoyan, 04. August, 2010.
 */

/*
 * Note that as of September 1, 2015, this file is controlled
 *  authoritatively in the git repository
 *    ssh://gitms@git.synaptics.com/git/biometrics/include.git.
 * Updates will continue to be copied into the CVSNT repository
 *  in /test/engineering/private/impl/micro/falcon/shared/vcsfw_v4vip.h.
 * The last authoritative version of this file in CVSNT was
 *   /test/engineering/private/impl/micro/falcon/shared/vcsfw_v4vip.h,v 1.7 2010/08/14 17:03:42 stonoyan
 * DO NOT EDIT THIS FILE IN THE CVSNT REPOSITORY.  Your changes will
 *  be overwritten.
 */

#ifndef __VCSFW_V4VIP_H
#define __VCSFW_V4VIP_H

#include "vcsPushPack1.h"

/* Definitions of cryptographic parameters */
#define VCSFW_VIP_AES_BLOCK_LEN   16 /* AES block length                    */
#define VCSFW_VIP_AES_128_KEY_LEN 16 /* Length of 256-bit AES key in bytes  */
#define VCSFW_VIP_AES_256_KEY_LEN 32 /* Length of 256-bit AES key in bytes  */
#define VCSFW_VIP_SHA1_HASH_LEN   20 /* Length of SHA1 hash and HMAC key    */
#define VCSFW_VIP_SHA256_HASH_LEN 32 /* Length of SHA256 hash and HMAC key  */

/* Bit-fields for specifying cryptographic algorithms and modes */
#define VCSFW_VIP_CFG_ALG_SHA1         0x01 /* SHA1 and HMAC-SHA1           */
#define VCSFW_VIP_CFG_ALG_SHA256       0x02 /* SHA256 and HMAC-SHA256       */
#define VCSFW_VIP_CFG_MODE_AES_CBC     0x04 /* Cipher Block Chaining mode   */
#define VCSFW_VIP_CFG_MODE_AES_CTR     0x08 /* Counter mode                 */

/* Common definitions */
#define VCSFW_VIP_TIMESTAMP_LEN   10 /* The length of decimal char string   */
                                     /* presenting Unix time in UTC format  */
#define VCSFW_VIP_NONCE_LEN       16 /* The length of nonce (pseudo-random  */
                                     /* sequence).                          */

/****************************************************************************/
/* VCSFW_CMD_VIP_CREATE_CLIENT_AUTH_TOKEN                                   */
/****************************************************************************/
/* COMMAND                                                                  */

#define VCSFW_VIP_MAX_APPID_LEN  128 /* The length of registered at VeriSign*/
                                     /* application ID string.              */

typedef struct VCS_PACKED vcsfw_cmd_vip_create_client_auth_token_s
{
    vcsUint8_t hash_algid;           /* Hash algorithm ID. See              */
                                     /* definitions with VCSFW_VIP_CFG_ALG_ */
                                     /* prefix above.                       */
    vcsUint32_t app_id_len;          /* The length of the Application ID    */  
    vcsUint8_t timestamp[VCSFW_VIP_TIMESTAMP_LEN]; /* UTC time in form of   */
                                                   /* dec character string  */
    vcsUint8_t app_id[VCSFW_VIP_MAX_APPID_LEN];    /* Application ID string */
} vcsfw_cmd_vip_create_client_auth_token_t;

/* REPLY                                                                    */
typedef struct VCS_PACKED vcsfw_reply_vip_create_client_auth_token_s
{
    vcsUint8_t token_len;                        /* Token length.           */
    vcsUint8_t token[VCSFW_VIP_SHA256_HASH_LEN]; /* The authentication token*/
    vcsUint8_t nonce[VCSFW_VIP_NONCE_LEN];       /* The nonce, generated in */
                                                 /* the sensor. */
} vcsfw_reply_vip_create_client_auth_token_t;

/****************************************************************************/
/* VCSFW_CMD_VIP_CREATE_TOTP_TOKEN                                          */
/****************************************************************************/
/* COMMAND                                                                  */

typedef struct VCS_PACKED vcsfw_cmd_vip_create_totp_token_s
{
    vcsUint8_t hash_algid;           /* Hash algorithm ID. See              */
                                     /* definitions with VCSFW_VIP_CFG_ALG_ */
                                     /* prefix above.                       */
    vcsUint8_t unused1[15];                        /* Just to make timestamp*/
                                                   /* 16-byte aligned       */
    vcsUint8_t timestamp[VCSFW_VIP_TIMESTAMP_LEN]; /* The timestamp in form */
                                                   /* of 64-bit integer in  */
                                                   /* big-endian byte order.*/
                                                   /* Last two bytes are    */
                                                   /* unused.*/
    vcsUint8_t unused2[6];                         /* Just to make eotp_seed*/
                                                   /* 16-byte aligned       */
    vcsUint8_t eotp_seed[VCSFW_VIP_SHA256_HASH_LEN + VCSFW_VIP_AES_BLOCK_LEN];
                                                   /* Encrypted OTP Seed    */
                                                   /* appended with IV:     */
                                                   /* E(SSEK,OTP_Seed)|IV   */
    vcsUint8_t otp_seed_mac[VCSFW_VIP_SHA256_HASH_LEN];
                                                   /* The HMAC-SHA256 MAC of*/
                                                   /* OTP Seed              */
} vcsfw_cmd_vip_create_totp_token_t;

/* REPLY                                                                    */
typedef struct VCS_PACKED vcsfw_reply_vip_create_totp_token_s
{
    vcsUint8_t token_len;                        /* Token length.           */
    vcsUint8_t token[VCSFW_VIP_SHA256_HASH_LEN]; /* The authentication token*/
} vcsfw_reply_vip_create_totp_token_t;

/****************************************************************************/
/* VCSFW_CMD_VIP_PROVISION_OTP_SEED                                         */
/****************************************************************************/
/* COMMAND                                                                  */

typedef struct VCS_PACKED vcsfw_cmd_vip_provision_otp_seed_s
{
    vcsUint8_t cfg_flags;            /* Bitfield for specifying:            */
                                     /* 1) Hash algorithm ID. See           */
                                     /* definitions with VCSFW_VIP_CFG_ALG_ */
                                     /* prefix above.                       */
                                     /* 2) AES mode. See definitions with   */
                                     /* VCSFW_VIP_CFG_MODE_AES_ prefix above*/
    vcsUint8_t nonce[VCSFW_VIP_NONCE_LEN];         /* Nonce (PRN)           */
    vcsUint8_t timestamp[VCSFW_VIP_TIMESTAMP_LEN]; /* UTC time in form of   */
                                                   /* dec character string  */
    vcsUint8_t unused[5];                          /* Just to make eotp_seed*/
                                                   /* 16-byte aligned       */
    vcsUint8_t eotp_seed[VCSFW_VIP_SHA256_HASH_LEN + VCSFW_VIP_AES_BLOCK_LEN];
                                                   /* Encrypted OTP Seed    */
                                                   /* appended with IV:     */
                                                   /* E(K_ENC_S,OTP_Seed)|IV*/
    vcsUint8_t otp_seed_mac[VCSFW_VIP_SHA256_HASH_LEN];
                                                   /* The HMAC-SHA256 MAC of*/
                                                   /* OTP Seed              */

} vcsfw_cmd_vip_provision_otp_seed_t;

/* REPLY                                                                    */
typedef struct VCS_PACKED vcsfw_reply_vip_provision_otp_seed_s
{
    vcsUint8_t eotp_seed[VCSFW_VIP_SHA256_HASH_LEN + VCSFW_VIP_AES_BLOCK_LEN];
                                                   /* Encrypted OTP Seed    */
                                                   /* appended with IV:     */
                                                   /* E(SSEK,OTP_Seed)|IV   */
} vcsfw_reply_vip_provision_otp_seed_t;

#include "vcsPopPack.h"

#endif      /* __VCSFW_V4VIP_H */
