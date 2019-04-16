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

#include <bcm_common_utils.h>
//#include <bcm-clients.h>
#include <gpio.h>
#include <drivers/pl022_spi.h>
#include <io.h>
#include <kernel/pseudo_ta.h>
#include <kernel/tee_time.h>
#include <mm/core_memprot.h>
#include <mm/core_mmu.h>
#include <pta_bcm_spi.h>
#include <pta_bcm_gpio.h>
#include <trace.h>

struct pl022_data pd;
uint8_t tx_buff[DATA_BUFF_SZ];
uint8_t rx_buff[DATA_BUFF_SZ];

static TEE_Result create_ta(void)
{
	DMSG("create entry point for static ta \"%s\"", SPI_TA_NAME);
	return TEE_SUCCESS;
}

static void destroy_ta(void)
{
	DMSG("destroy entry point for static ta \"%s\"", SPI_TA_NAME);
}

static TEE_Result open_session(uint32_t nParamTypes __unused,
			       TEE_Param pParams[4] __unused,
			       void **ppSessionContext __unused)
{
	DMSG("open entry point for static ta \"%s\"", SPI_TA_NAME);
	return TEE_SUCCESS;
}

static void close_session(void *pSessionContext __unused)
{
	DMSG("close entry point for static ta \"%s\"", SPI_TA_NAME);
}

static void pta_spi_cs_mux(void)
{
	paddr_t phys_addr = SPI_0_CS_MUX_PAD;
	vaddr_t virt_addr = (vaddr_t)phys_to_virt(phys_addr, MEM_AREA_IO_SEC);
	uint32_t val;

	val = io_read32(virt_addr);
	val &= ~PIN_MUX_FUNC_MASK;
	val |= PIN_MUX_FUNC_GPIO;
	io_write32(virt_addr, val);
}

static void pta_spi_cs_cb(enum gpio_level value)
{
	gpio_set_value(pd.cs_data.gd, value);
	tee_time_busy_wait(SPI_FLASH_CMD_COMPLETE_MS);
}

/*
 * Configures SPI
 * Takes three TEE parameter
 * Param 1:
 *	mode and phase = Params[0].value.a
 *	speed = Params[0].value.b
 * Param 2:
 *	data size bits = Params[1].value.a
 *	loop back mode = Params[1].value.b
 * Param 3:
 *	Chip select GPIO = Params[2].value.a
 */
static TEE_Result pta_spi_config(uint32_t nParamTypes,
				 TEE_Param pParams[TEE_NUM_PARAMS])
{
	vaddr_t spi_base = (vaddr_t)phys_to_virt(SPI_0_BASE, MEM_AREA_IO_SEC);
	uint32_t mode;
	uint32_t speed;
	uint32_t data_size;
	uint32_t loopback;
	uint32_t gpio_num;
	uint32_t exp_param_types = TEE_PARAM_TYPES(TEE_PARAM_TYPE_VALUE_INPUT,
						   TEE_PARAM_TYPE_VALUE_INPUT,
						   TEE_PARAM_TYPE_VALUE_INPUT,
						   TEE_PARAM_TYPE_NONE);

	if (exp_param_types != nParamTypes) {
		EMSG("%s(): Invalid Param types\n", __func__);
		return TEE_ERROR_BAD_PARAMETERS;
	}

	/* Valid modes are 0, 1, 2 and 3 */
	mode = (pParams[0].value.a & 0x3);
	speed = pParams[0].value.b;
	if (speed > (SPI_0_CLK_HZ / 2)) {
		EMSG("%s(): Bad Speed (%d) request\n", __func__, speed);
		return TEE_ERROR_BAD_PARAMETERS;
	}

	data_size = pParams[1].value.a;
	loopback = pParams[1].value.b;

	/* Non loopback mode, configure GPIO */
	if (!loopback) {
		gpio_num = pParams[2].value.a;

		/* Configure the CS pin to GPIO */
		pta_spi_cs_mux();

		pd.cs_data.gd = request_gpiod(gpio_num, &pd);
		if (!pd.cs_data.gd) {
			EMSG("%s(): gpio client data alloc failed\n", __func__);
			return TEE_ERROR_GENERIC;
		}
		
		/* Set GPIO to output with default value to 1 */
		gpio_set_direction(pd.cs_data.gd, GPIO_DIR_OUT);

		/* Chip control is done manually */
		pd.cs_control = PL022_CS_CTRL_AUTO_GPIO;
	}

	pd.base = spi_base;
	pd.clk_hz = SPI_0_CLK_HZ;
	pd.speed_hz = speed;
	pd.mode = mode;
	pd.data_size_bits = data_size;
	pd.loopback = loopback;

	/* Initialize SPI controller */
	pl022_init(&pd);
	pd.chip.ops->configure(&(pd.chip));
	pd.chip.ops->start(&(pd.chip));

	return TEE_SUCCESS;
}

/*
 * Macronix Flash Read identification.
 * RDID instruction is for reading the manufacturer ID of 1-byte
 * and followed by Device ID of 2-byte. Macronix Manufacturer ID is
 * 0xC2(hex), memory type ID is 0x20(hex) as the first-byte device ID,
 * and individual device ID of second-byte ID is 10(hex) for MX25L512E.
 */
static TEE_Result pta_spi_rdid(
			uint32_t nParamTypes __unused,
			TEE_Param pParams[TEE_NUM_PARAMS] __unused)
{
	uint8_t tx[SPI_FLASH_RDID_SZ] = {0};
	uint8_t rx[SPI_FLASH_RDID_SZ] = {0};
	enum spi_result res;
	uint32_t i;

	/* Check whether pta_bcm_spi_config() is called */
	if (!(pd.base && pd.data_size_bits) || pd.loopback) {
		EMSG("%s(): SPI is not initialized\n", __func__);
		return TEE_ERROR_BAD_STATE;
	}

	tx[0] = MACRONIX_FLASH_RDID;

	/* Make sure Chip select is LOW before SPI Transactions */
	//pta_spi_cs_cb(GPIO_LEVEL_LOW);
	tee_time_busy_wait(SPI_FLASH_CS_LOW_MS);
	res = pd.chip.ops->txrx8(&(pd.chip), tx, rx, 1);
	if (res) {
		EMSG("SPI transceive error sending RDID %d", res);
		return TEE_ERROR_COMMUNICATION;
	}

	/*
	 * As we are operting at low speed, give enough time for the slave
	 * to prepare data.
	 */
	tee_time_busy_wait(SPI_FLASH_DATA_PREP_MS);
	tx[0] = 0;
	for (i = 0; i < SPI_FLASH_RDID_SZ; i++) {
		res = pd.chip.ops->txrx8(&(pd.chip), &tx[0], &rx[i], 1);
		if (res) {
			EMSG("SPI transceive error Reading RDID %d", res);
			return TEE_ERROR_COMMUNICATION;
		}
	}
	/* Make sure Chip select is High after SPI Transactions */
//	pta_spi_cs_cb(GPIO_LEVEL_HIGH);
	pd.chip.ops->end(&pd.chip);

	for (i = 0; i < SPI_FLASH_RDID_SZ; i++)
		IMSG("RDID [%d] = 0x%02x\n", i, rx[i]);

	return TEE_SUCCESS;
}

/* Erases the complete chip */
static TEE_Result pta_spi_erase(
			uint32_t nParamTypes __unused,
			TEE_Param pParams[TEE_NUM_PARAMS] __unused)
{
	uint8_t tx[SPI_FLASH_ERASE_CMD_SZ] = {0};
	uint8_t rx[1] = {0};
	int timeout = SPI_FLASH_CMD_ITER_CNT;
	enum spi_result res;

	/* Write enable operation */
	tx[0] = MACRONIX_FLASH_WE;
	pd.chip.ops->start(&pd.chip);

	/* Make sure Chip select is LOW before SPI Transactions */
//	pta_spi_cs_cb(GPIO_LEVEL_LOW);
	tee_time_busy_wait(SPI_FLASH_CS_LOW_MS);
	res = pd.chip.ops->txrx8(&(pd.chip), tx, rx, 1);
	if (res) {
		EMSG("SPI transceive error flash write enable %d", res);
		return TEE_ERROR_GENERIC;
	}
	/* Make sure Chip select is High after SPI Transactions */
	pd.chip.ops->end(&pd.chip);

//	pta_spi_cs_cb(GPIO_LEVEL_HIGH);
	tee_time_busy_wait(SPI_FLASH_CMD_COMPLETE_MS);

	/* Chip Erase operation */
	tx[0] = MACRONIX_FLASH_CE;

	/* Make sure Chip select is LOW before SPI Transactions */
	//pta_spi_cs_cb(GPIO_LEVEL_LOW);
	pd.chip.ops->start(&pd.chip);

	tee_time_busy_wait(SPI_FLASH_CS_LOW_MS);
	res = pd.chip.ops->txrx8(&(pd.chip), tx, rx, 1);
	if (res) {
		EMSG("SPI transceive error flash chip erase %d", res);
		return TEE_ERROR_GENERIC;
	}
	/* Make sure Chip select is High after SPI Transactions */
	pd.chip.ops->end(&pd.chip);

//	pta_spi_cs_cb(GPIO_LEVEL_HIGH);
	tee_time_busy_wait(SPI_FLASH_CMD_COMPLETE_MS);

	/*
	 * Read the Read Status Register (RDSR)
	 * Need to wait till "Write in Progress"(WIP) bit is 0
	 */
	tx[0] = MACRONIX_FLASH_RDSR;
	tx[1] = 0x0;
	do {
		pta_spi_cs_cb(GPIO_LEVEL_LOW);
		tee_time_busy_wait(SPI_FLASH_CS_LOW_MS);
		res = pd.chip.ops->txrx8(&(pd.chip), tx, rx, 1);
		if (res) {
			EMSG("SPI transceive error sending RDSR %d", res);
			return TEE_ERROR_GENERIC;
		}
		res = pd.chip.ops->txrx8(&(pd.chip), &tx[1], rx, 1);
		if (res) {
			EMSG("SPI transceive error reading RDSR %d", res);
			return TEE_ERROR_GENERIC;
		}
		pta_spi_cs_cb(GPIO_LEVEL_HIGH);
		tee_time_busy_wait(SPI_FLASH_CS_LOW_MS);
	} while ((rx[0] & MACRONIX_FLASH_RDSR_WIP_BIT) && timeout--);

	if (timeout == 0) {
		EMSG("SPI transceive error Timeout for flash chip erase");
		return TEE_ERROR_GENERIC;
	}

	IMSG("Chip erase is successful\n");
	return TEE_SUCCESS;
}

/* Read DATA_BUFF_SZ bytes of data starting from 0x0 loation to rx_buff[] */
static TEE_Result pta_spi_read(
			uint32_t nParamTypes __unused,
			TEE_Param pParams[TEE_NUM_PARAMS] __unused)
{
	uint8_t tx;
	uint8_t rx = 0;
	uint8_t temp;
	int i;
	int timeout = SPI_FLASH_CMD_ITER_CNT;
	enum spi_result res;

	tx = MACRONIX_FLASH_RDSR;
	temp = 0x0;
	tee_time_busy_wait(SPI_FLASH_CS_LOW_MS);
	do {
		pta_spi_cs_cb(GPIO_LEVEL_LOW);
		tee_time_busy_wait(SPI_FLASH_CS_LOW_MS);
		res = pd.chip.ops->txrx8(&(pd.chip), &tx, &rx, 1);
		if (res) {
			EMSG("SPI transceive error sending RDSR %d", res);
			return TEE_ERROR_GENERIC;
		}
		res = pd.chip.ops->txrx8(&(pd.chip), &temp, &rx, 1);
		if (res) {
			EMSG("SPI transceive error reading RDSR %d", res);
			return TEE_ERROR_GENERIC;
		}
		pta_spi_cs_cb(GPIO_LEVEL_HIGH);
		tee_time_busy_wait(SPI_FLASH_CS_LOW_MS);
	} while ((rx & MACRONIX_FLASH_RDSR_WIP_BIT) && timeout--);

	if (timeout == 0) {
		EMSG("SPI transceive error Timeout for flash Read");
		return TEE_ERROR_GENERIC;
	}

	tee_time_busy_wait(SPI_FLASH_CMD_COMPLETE_MS);

	/* Read Data operation */
	tx = MACRONIX_FLASH_RD;
	/* Make sure Chip select is LOW before SPI Transactions */
	pta_spi_cs_cb(GPIO_LEVEL_LOW);
	tee_time_busy_wait(SPI_FLASH_CS_LOW_MS);
	res = pd.chip.ops->txrx8(&(pd.chip), &tx, &rx, 1);
	if (res) {
		EMSG("SPI transceive error flash read data operation %d", res);
		return TEE_ERROR_GENERIC;
	}

	/* 24 bit Address */
	tx = 0x0;
	res = pd.chip.ops->txrx8(&(pd.chip), &tx, &rx, 1);
	if (res) {
		EMSG("SPI transceive error flash read data operation %d", res);
		return TEE_ERROR_GENERIC;
	}
	tee_time_busy_wait(SPI_FLASH_DATA_PREP_MS);

	res = pd.chip.ops->txrx8(&(pd.chip), &tx, &rx, 1);
	if (res) {
		EMSG("SPI transceive error flash read data operation %d", res);
		return TEE_ERROR_GENERIC;
	}
	tee_time_busy_wait(SPI_FLASH_DATA_PREP_MS);

	res = pd.chip.ops->txrx8(&(pd.chip), &tx, &rx, 1);
	if (res) {
		EMSG("SPI transceive error flash read data operation %d", res);
		return TEE_ERROR_GENERIC;
	}
	tee_time_busy_wait(SPI_FLASH_DATA_PREP_MS);

	/* Reading Data from Flash */
	tx = 0x0;
	for (i = 0; i < DATA_BUFF_SZ; i++) {
		tee_time_busy_wait(SPI_FLASH_INCREMENT_RD_MS);
		res = pd.chip.ops->txrx8(&(pd.chip), &tx, &rx_buff[i], 1);
		if (res) {
			EMSG("SPI transceive error flash read data %d", res);
			return TEE_ERROR_GENERIC;
		}
	}

	tee_time_busy_wait(SPI_FLASH_CMD_COMPLETE_MS);
	pta_spi_cs_cb(GPIO_LEVEL_HIGH);
	for (i = 0; i < DATA_BUFF_SZ; ) {
		IMSG("%02x  %02x  %02x  %02x  %02x  %02x  %02x  %02x\n",
			rx_buff[i], rx_buff[i+1], rx_buff[i+2], rx_buff[i+3],
			rx_buff[i+4], rx_buff[i+5], rx_buff[i+6], rx_buff[i+7]);
		i = i + 8;
	}

	return TEE_SUCCESS;
}

/* Write DATA_BUFF_SZ bytes of data from tx_buff[] to 0x0 loation */
static TEE_Result pta_spi_write(
			uint32_t nParamTypes __unused,
			TEE_Param pParams[TEE_NUM_PARAMS] __unused)
{
	uint8_t tx;
	uint8_t rx = 0;
	uint8_t temp;
	int i;
	int timeout = SPI_FLASH_CMD_ITER_CNT;
	enum spi_result res;

	/* Write enable operation */
	tx = MACRONIX_FLASH_WE;

	/* Make sure Chip select is LOW before SPI Transactions */
//	pta_spi_cs_cb(GPIO_LEVEL_LOW);
	pd.chip.ops->start(&pd.chip);

	tee_time_busy_wait(SPI_FLASH_CS_LOW_MS);
	res = pd.chip.ops->txrx8(&(pd.chip), &tx, &rx, 1);
	if (res) {
		EMSG("SPI transceive error flash write enable %d", res);
		return TEE_ERROR_GENERIC;
	}
	/* Make sure Chip select is High after SPI Transactions */
//	pta_spi_cs_cb(GPIO_LEVEL_HIGH);
	pd.chip.ops->end(&pd.chip);

	tee_time_busy_wait(SPI_FLASH_CMD_COMPLETE_MS);

	/* Write Data operation */
	tx = MACRONIX_FLASH_WT;
	/* Make sure Chip select is LOW before SPI Transactions */
//	pta_spi_cs_cb(GPIO_LEVEL_LOW);
	pd.chip.ops->start(&pd.chip);

	tee_time_busy_wait(SPI_FLASH_CS_LOW_MS);
	res = pd.chip.ops->txrx8(&(pd.chip), &tx, &rx, 1);
	if (res) {
		EMSG("SPI transceive error flash write data operation %d", res);
		return TEE_ERROR_GENERIC;
	}

	/* 24 bit Address */
	tx = 0x0;
	res = pd.chip.ops->txrx8(&(pd.chip), &tx, &rx, 1);
	if (res) {
		EMSG("SPI transceive error flash write data operation %d", res);
		return TEE_ERROR_GENERIC;
	}
	tee_time_busy_wait(SPI_FLASH_DATA_PREP_MS);

	res = pd.chip.ops->txrx8(&(pd.chip), &tx, &rx, 1);
	if (res) {
		EMSG("SPI transceive error flash write data operation %d", res);
		return TEE_ERROR_GENERIC;
	}
	tee_time_busy_wait(SPI_FLASH_DATA_PREP_MS);

	res = pd.chip.ops->txrx8(&(pd.chip), &tx, &rx, 1);
	if (res) {
		EMSG("SPI transceive error flash write data operation %d", res);
		return TEE_ERROR_GENERIC;
	}
	tee_time_busy_wait(SPI_FLASH_DATA_PREP_MS);

	/* Fill inremental pattern in TX buff */
	for (i = 0; i < DATA_BUFF_SZ; i++)
		tx_buff[i] = i;

	/* Write Data to Flash */
	for (i = 0; i < DATA_BUFF_SZ; i++) {
		tee_time_busy_wait(SPI_FLASH_INCREMENT_RD_MS);
		res = pd.chip.ops->txrx8(&(pd.chip), &tx_buff[i], &rx, 1);
		if (res) {
			EMSG("SPI transceive error flash write data %d", res);
			return TEE_ERROR_GENERIC;
		}
	}

	tee_time_busy_wait(SPI_FLASH_CMD_COMPLETE_MS);
//	pta_spi_cs_cb(GPIO_LEVEL_HIGH);
	pd.chip.ops->end(&pd.chip);

	tx = MACRONIX_FLASH_RDSR;
	temp = 0x0;
	tee_time_busy_wait(SPI_FLASH_CS_LOW_MS);
	do {
		pd.chip.ops->start(&pd.chip);

	//	pta_spi_cs_cb(GPIO_LEVEL_LOW);
		tee_time_busy_wait(SPI_FLASH_CS_LOW_MS);
		res = pd.chip.ops->txrx8(&(pd.chip), &tx, &rx, 1);
		if (res) {
			EMSG("SPI transceive error sending RDSR %d", res);
			return TEE_ERROR_GENERIC;
		}
		res = pd.chip.ops->txrx8(&(pd.chip), &temp, &rx, 1);
		if (res) {
			EMSG("SPI transceive error reading RDSR %d", res);
			return TEE_ERROR_GENERIC;
		}
		pd.chip.ops->end(&pd.chip);

//	pta_spi_cs_cb(GPIO_LEVEL_HIGH);
		tee_time_busy_wait(SPI_FLASH_CS_LOW_MS);
	} while ((rx & MACRONIX_FLASH_RDSR_WIP_BIT) && timeout--);

	tee_time_busy_wait(SPI_FLASH_CMD_COMPLETE_MS);

	if (timeout == 0) {
		EMSG("SPI transceive error Timeout for flash Write");
		return TEE_ERROR_BUSY;
	}

	return TEE_SUCCESS;
}

/* Does erase, read, write and verify data */
static TEE_Result pta_spi_flash_test(
			uint32_t nParamTypes __unused,
			TEE_Param pParams[TEE_NUM_PARAMS] __unused)
{
	TEE_Result res = TEE_SUCCESS;
	int i;

	res = pta_spi_erase(nParamTypes, pParams);
	if (res != TEE_SUCCESS) {
		EMSG("%s(): pta_spi_erase failed\n", __func__);
		return res;
	}

	/* Fill Rx_buff with 0 */
	memset(rx_buff, 0, DATA_BUFF_SZ);

	res = pta_spi_write(nParamTypes, pParams);
	if (res != TEE_SUCCESS) {
		EMSG("%s(): pta_spi_write failed\n", __func__);
		return res;
	}

	res = pta_spi_read(nParamTypes, pParams);
	if (res != TEE_SUCCESS) {
		EMSG("%s(): pta_spi_read failed\n", __func__);
		return res;
	}

	for (i = 0; i < DATA_BUFF_SZ; i++) {
		if (tx_buff[i] != rx_buff[i]) {
			IMSG("Data mismatch tx[%d] 0x%02x rx[%d] 0x%02x\n",
				i, tx_buff[i], i, rx_buff[i]);
			res = TEE_ERROR_GENERIC;
		}
	}
	return res;
}

static TEE_Result invoke_command(void *pSessionContext __unused,
				 uint32_t nCommandID,
				 uint32_t nParamTypes,
				 TEE_Param pParams[TEE_NUM_PARAMS])
{
	TEE_Result res = TEE_SUCCESS;

	DMSG("command entry point[%d] for \"%s\"", nCommandID, SPI_TA_NAME);

	switch (nCommandID) {
	case PTA_BCM_SPI_CMD_CFG:
		res = pta_spi_config(nParamTypes, pParams);
		break;
	case PTA_BCM_SPI_CMD_RDID:
		res = pta_spi_rdid(nParamTypes, pParams);
		break;
	case PTA_BCM_SPI_CMD_ERASE:
		res = pta_spi_erase(nParamTypes, pParams);
		break;
	case PTA_BCM_SPI_CMD_READ:
		res = pta_spi_read(nParamTypes, pParams);
		break;
	case PTA_BCM_SPI_CMD_WRITE:
		res = pta_spi_write(nParamTypes, pParams);
		break;
	case PTA_BCM_SPI_CMD_FLASH_TEST:
		res = pta_spi_flash_test(nParamTypes, pParams);
		break;
	default:
		EMSG("%d Not supported %s\n", nCommandID, SPI_TA_NAME);
		res = TEE_ERROR_NOT_SUPPORTED;
		break;
	}

	return res;
}

pseudo_ta_register(.uuid = SPI_SERVICE_UUID,
		   .name = SPI_TA_NAME,
		   .flags = PTA_DEFAULT_FLAGS,
		   .create_entry_point = create_ta,
		   .destroy_entry_point = destroy_ta,
		   .open_session_entry_point = open_session,
		   .close_session_entry_point = close_session,
		   .invoke_command_entry_point = invoke_command);
