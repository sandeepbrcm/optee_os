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

#include <io.h>
#include <kernel/pseudo_ta.h>
#include <drivers/bcm_sotp.h>
#include <pta_bcm_sotp.h>

static TEE_Result create_ta(void)
{
	DMSG("create entry point for static ta \"%s\"", SOTP_TA_NAME);
	return TEE_SUCCESS;
}

static void destroy_ta(void)
{
	DMSG("destroy entry point for static ta \"%s\"", SOTP_TA_NAME);
}

static TEE_Result open_session(uint32_t nParamTypes __unused,
			       TEE_Param pParams[4] __unused,
			       void **ppSessionContext __unused)
{
	DMSG("open entry point for static ta \"%s\"", SOTP_TA_NAME);
	return TEE_SUCCESS;
}

static void close_session(void *pSessionContext __unused)
{
	DMSG("close entry point for static ta \"%s\"", SOTP_TA_NAME);
}

static TEE_Result pta_sotp_read(
			uint32_t nParamTypes,
			TEE_Param pParams[TEE_NUM_PARAMS])
{
	uint64_t sotp_row_value = 0;
	uint32_t val = 0;
	vaddr_t *sotp_base_vaddr = 0;
	uint32_t exp_param_types = TEE_PARAM_TYPES(
			TEE_PARAM_TYPE_VALUE_INPUT,
			TEE_PARAM_TYPE_NONE,
			TEE_PARAM_TYPE_NONE,
			TEE_PARAM_TYPE_NONE);
	if (exp_param_types != nParamTypes) {
		EMSG("%s(): Invalid Param types\n", __func__);
		return TEE_ERROR_BAD_PARAMETERS;
	}
	val = pParams[0].value.a;

//	sotp_base_vaddr = (vaddr_t *)bcm_iproc_sotp_init();

	if (sotp_base_vaddr) {
		bcm_iproc_sotp_mem_read(val, 1,
				&sotp_row_value);
		IMSG("OTP value for row:%d - %llx", val, sotp_row_value);
	}

	return TEE_SUCCESS;
}

static TEE_Result pta_sotp_write(
			uint32_t nParamTypes __unused,
			TEE_Param pParams[TEE_NUM_PARAMS] __unused)
{
	/* Noting as of now */
	return TEE_ERROR_NOT_IMPLEMENTED;
}

static TEE_Result invoke_command(void *pSessionContext __unused,
				 uint32_t nCommandID,
				 uint32_t nParamTypes,
				 TEE_Param pParams[TEE_NUM_PARAMS])
{
	TEE_Result res = TEE_SUCCESS;

	DMSG("command entry point[%d] for \"%s\"", nCommandID, SOTP_TA_NAME);

	switch (nCommandID) {
	case PTA_BCM_SOTP_CMD_READ:
		res = pta_sotp_read(nParamTypes, pParams);
		break;
	case PTA_BCM_SOTP_CMD_WRITE:
		res = pta_sotp_write(nParamTypes, pParams);
		break;
	default:
		EMSG("%d Not supported %s\n", nCommandID, SOTP_TA_NAME);
		res = TEE_ERROR_NOT_SUPPORTED;
		break;
	}

	return res;
}

pseudo_ta_register(.uuid = SOTP_SERVICE_UUID,
		   .name = SOTP_TA_NAME,
		   .flags = PTA_DEFAULT_FLAGS,
		   .create_entry_point = create_ta,
		   .destroy_entry_point = destroy_ta,
		   .open_session_entry_point = open_session,
		   .close_session_entry_point = close_session,
		   .invoke_command_entry_point = invoke_command);
