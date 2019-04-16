/*
 * Copyright 2019 Broadcom
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
#include <io.h>
#include <kernel/pseudo_ta.h>
#include <pta_bcm_gpio.h>
#include <trace.h>
#include <gpio.h>

static char owner_name[10] = "ptagpio\n";
static void *owner = (void*)&name[0];

static TEE_Result create_ta(void)
{
	DMSG("create entry point for static ta \"%s\"", GPIO_TA_NAME);
	return TEE_SUCCESS;
}

static void destroy_ta(void)
{
	DMSG("destroy entry point for static ta \"%s\"", GPIO_TA_NAME);
}

static TEE_Result open_session(uint32_t nParamTypes __unused,
		TEE_Param pParams[4] __unused, void **ppSessionContext __unused)
{
	DMSG("open entry point for static ta \"%s\"", GPIO_TA_NAME);
	return TEE_SUCCESS;
}

static void close_session(void *pSessionContext __unused)
{
	DMSG("close entry point for static ta \"%s\"", GPIO_TA_NAME);
}

/*
 * Configures pin num to GPIO
 * Takes one TEE parameter
 * GPIO_number = Params[0].value.a
 * GPIO_direction = Params[0].value.b
 * If GPIO_direction is "1", its output, else
 * GPIO_direction is input
 */
static TEE_Result pta_gpio_config(uint32_t nParamTypes,
				  TEE_Param pParams[TEE_NUM_PARAMS])
{
	uint32_t gpio_num;
	uint32_t val;
	TEE_Result res;
	struct gpio_desc *gd;
	uint32_t exp_param_types = TEE_PARAM_TYPES(TEE_PARAM_TYPE_VALUE_INPUT,
					TEE_PARAM_TYPE_NONE,
					TEE_PARAM_TYPE_NONE,
					TEE_PARAM_TYPE_NONE);

	if (exp_param_types != nParamTypes) {
		EMSG("%s(): Invalid Param types\n", __func__);
		return TEE_ERROR_BAD_PARAMETERS;
	}

	gpio_num = pParams[0].value.a;
	val = pParams[0].value.b;
	gd = request_gpiod(gpio_num, dev); 

/*	gpio_data = bcm_alloc_gpio_client(gpio_num);
	if (!gpio_data) {
		EMSG("%s(): gpio client data alloc failed\n", __func__);
		return TEE_ERROR_GENERIC;
	}
	gpio_data->gpiopin = gpio_num;
*/
	if (val == 1)
		/* Set GPIO to output with default value to 0 */
		gpio_set_direction(gd, GPIO_DIR_OUT);
	else
		gpio_set_direction(gd, GPIO_DIR_IN);

	release_gpiod(gd);

	return 0;
}

static TEE_Result pta_gpio_set(uint32_t nParamTypes,
			       TEE_Param pParams[TEE_NUM_PARAMS])
{
	uint32_t gpio_num;
	uint32_t val;
	TEE_Result res;
	struct gpio_desc *gd;
	uint32_t exp_param_types = TEE_PARAM_TYPES(TEE_PARAM_TYPE_VALUE_INPUT,
					TEE_PARAM_TYPE_NONE,
					TEE_PARAM_TYPE_NONE,
					TEE_PARAM_TYPE_NONE);

	if (exp_param_types != nParamTypes) {
		EMSG("%s(): Invalid Param types\n", __func__);
		return TEE_ERROR_BAD_PARAMETERS;
	}

	gpio_num = pParams[0].value.a;
	val = pParams[0].value.b;

	val = val ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW;

	gd = request_gpiod(gpio_num, dev);
	/*
	 * for setting a value to GPIO Pin,
	 * need to make sure the PIN is configured in
	 * output direction.
	 */
	gpio_set_direction(gd, GPIO_DIR_OUT);

	gpio_set_value(gd, val);
	DMSG("GPIO(%d) value = 0x%08x\n", gpio_num, gpio_get_value(gd));
	release_gpiod(gd);

	return TEE_SUCCESS;
}

static TEE_Result pta_gpio_get(uint32_t nParamTypes,
			       TEE_Param pParams[TEE_NUM_PARAMS])
{
	struct gpio_desc *gd;
	uint32_t gpio_num;
	uint32_t *ret_val;
	uint32_t exp_param_types = TEE_PARAM_TYPES(TEE_PARAM_TYPE_VALUE_INPUT,
					TEE_PARAM_TYPE_VALUE_OUTPUT,
					TEE_PARAM_TYPE_NONE,
					TEE_PARAM_TYPE_NONE);

	if (exp_param_types != nParamTypes) {
		EMSG("%s(): Invalid Param types\n", __func__);
		return TEE_ERROR_BAD_PARAMETERS;
	}

	gpio_num = pParams[0].value.a;
	ret_val = (uint32_t *)&pParams[1].value.a;

	gd = request_gpiod(gpio_num, dev);
	*ret_val = gpio_get_value(gd);
	release_gpiod(gd);
	DMSG("gpio(%d) value = 0x%08x\n", gpio_num, *ret_val);
	return TEE_SUCCESS;
}

static TEE_Result invoke_command(void *pSessionContext __unused,
				 uint32_t nCommandID,
				 uint32_t nParamTypes,
				 TEE_Param pParams[TEE_NUM_PARAMS])
{
	TEE_Result res = TEE_SUCCESS;

	DMSG("command entry point[%d] for \"%s\"", nCommandID, GPIO_TA_NAME);

	switch (nCommandID) {
	case PTA_BCM_GPIO_CMD_CFG:
		res = pta_gpio_config(nParamTypes, pParams);
		break;
	case PTA_BCM_GPIO_CMD_SET:
		res = pta_gpio_set(nParamTypes, pParams);
		break;
	case PTA_BCM_GPIO_CMD_GET:
		res = pta_gpio_get(nParamTypes, pParams);
		break;
	default:
		EMSG("%d Not supported %s\n", nCommandID, GPIO_TA_NAME);
		res = TEE_ERROR_NOT_SUPPORTED;
		break;
	}

	return res;
}

pseudo_ta_register(.uuid = GPIO_SERVICE_UUID,
		   .name = GPIO_TA_NAME,
		   .flags = PTA_DEFAULT_FLAGS,
		   .create_entry_point = create_ta,
		   .destroy_entry_point = destroy_ta,
		   .open_session_entry_point = open_session,
		   .close_session_entry_point = close_session,
		   .invoke_command_entry_point = invoke_command);
