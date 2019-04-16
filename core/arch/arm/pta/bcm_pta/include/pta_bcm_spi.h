/*
 * Copyright 2018-2019 Broadcom
 *
 * This program is the proprietary software of Broadcom and/or
 * its licensors, and may only be used, duplicated, modified or distributed
 * pursuant to the terms and conditions of a separate, written license
 * agreement executed between you and Broadcom (an "Authorized License").
 * Except as set forth in an Authorized License, Broadcom grants no license
 * (express or implied), right to use, or waiver of any kind with respect to
 * the Software, and Broadcom expressly reserves all rights in and to the
 * Software and all intellectual property rights therein.  IF YOU HAVE NO
 * AUTHORIZED LICENSE, THEN YOU HAVE NO RIGHT TO USE THIS SOFTWARE IN ANY
 * WAY, AND SHOULD IMMEDIATELY NOTIFY BROADCOM AND DISCONTINUE ALL USE OF
 * THE SOFTWARE.
 *
 * Except as expressly set forth in the Authorized License,
 *
 * 1. This program, including its structure, sequence and organization,
 * constitutes the valuable trade secrets of Broadcom, and you shall use
 * all reasonable efforts to protect the confidentiality thereof, and to
 * use this information only in connection with your use of Broadcom
 * integrated circuit products.
 *
 * 2. TO THE MAXIMUM EXTENT PERMITTED BY LAW, THE SOFTWARE IS PROVIDED
 * "AS IS" AND WITH ALL FAULTS AND BROADCOM MAKES NO PROMISES,
 * REPRESENTATIONS OR WARRANTIES, EITHER EXPRESS, IMPLIED, STATUTORY, OR
 * OTHERWISE, WITH RESPECT TO THE SOFTWARE.  BROADCOM SPECIFICALLY
 * DISCLAIMS ANY AND ALL IMPLIED WARRANTIES OF TITLE, MERCHANTABILITY,
 * NONINFRINGEMENT, FITNESS FOR A PARTICULAR PURPOSE, LACK OF VIRUSES,
 * ACCURACY OR COMPLETENESS, QUIET ENJOYMENT, QUIET POSSESSION OR
 * CORRESPONDENCE TO DESCRIPTION. YOU ASSUME THE ENTIRE RISK ARISING
 * OUT OF USE OR PERFORMANCE OF THE SOFTWARE.
 *
 * 3. TO THE MAXIMUM EXTENT PERMITTED BY LAW, IN NO EVENT SHALL
 * BROADCOM OR ITS LICENSORS BE LIABLE FOR (i) CONSEQUENTIAL, INCIDENTAL,
 * SPECIAL, INDIRECT, OR EXEMPLARY DAMAGES WHATSOEVER ARISING OUT OF OR
 * IN ANY WAY RELATING TO YOUR USE OF OR INABILITY TO USE THE SOFTWARE EVEN
 * IF BROADCOM HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES; OR (ii)
 * ANY AMOUNT IN EXCESS OF THE AMOUNT ACTUALLY PAID FOR THE SOFTWARE ITSELF
 * OR U.S. $1, WHICHEVER IS GREATER. THESE LIMITATIONS SHALL APPLY
 * NOTWITHSTANDING ANY FAILURE OF ESSENTIAL PURPOSE OF ANY LIMITED REMEDY.
 */

#ifndef PTA_BCM_SPI_H
#define PTA_BCM_SPI_H

#define SPI_SERVICE_UUID \
		{ 0x6272636D, 0x2018, 0x1101,  \
		{ 0x42, 0x43, 0x4D, 0x5F, 0x53, 0x50, 0x49, 0x30 } }

/*
 * Following commands are for SPI FLASH device.
 * This can be extended to the other SPI Slaves
 */
enum {
	PTA_BCM_SPI_CMD_CFG = 0,
	/* Read identification Register */
	PTA_BCM_SPI_CMD_RDID,
	/* Chip erase */
	PTA_BCM_SPI_CMD_ERASE,
	PTA_BCM_SPI_CMD_READ,
	PTA_BCM_SPI_CMD_WRITE,
	PTA_BCM_SPI_CMD_FLASH_TEST,
} pta_bcm_spi_cmd;

#define PIN_MUX_FUNC_MASK	0x7
#define PIN_MUX_FUNC_GPIO	0x3

/* SPI Flash commands */
#define MACRONIX_FLASH_WT	0x02 /* Write Data */
#define MACRONIX_FLASH_RD	0x03 /* Read Data */
#define MACRONIX_FLASH_WD	0x04 /* Write Disable*/
#define MACRONIX_FLASH_RDSR	0x05 /* Read Status Register */
#define MACRONIX_FLASH_WE	0x06 /* Write Enable */
#define MACRONIX_FLASH_CE	0xC7 /* Chip Erase */
#define MACRONIX_FLASH_RDID	0x9F /* Read Identification */

/* Write in Progress bit */
#define MACRONIX_FLASH_RDSR_WIP_BIT	0x1

/* buffer size for read and write operations */
#define DATA_BUFF_SZ			256

#define SPI_FLASH_CS_LOW_MS		2
/* Time needed to prepare the data after sending the command */
#define SPI_FLASH_DATA_PREP_MS		20
#define SPI_FLASH_INCREMENT_RD_MS	10
#define SPI_FLASH_CMD_COMPLETE_MS	200

/* MAX Iteration count for the command success check */
#define SPI_FLASH_CMD_ITER_CNT		100

/* Flash commands data sizes */
#define SPI_FLASH_ERASE_CMD_SZ		2
#define SPI_FLASH_RDID_SZ		3

#define SPI_TA_NAME		"pta_bcm_spi.ta"
#endif /* PTA_BCM_SPI_H */
