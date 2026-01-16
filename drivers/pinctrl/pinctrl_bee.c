/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pinctrl.h>

#include <rtl876x_pinmux.h>

static void pinctrl_configure_pin(const pinctrl_soc_pin_t *pin)
{
	uint32_t cfg_fun = pin[0].fun;
	uint32_t cfg_pin = pin[0].pin;
	uint32_t cfg_dir = pin[0].dir ? PAD_OUT_ENABLE : PAD_OUT_DISABLE;
	uint32_t cfg_drv = pin[0].drive ? PAD_OUT_HIGH : PAD_OUT_LOW;
	uint32_t cfg_pull = pin[0].pull_dis ? PAD_PULL_NONE :
						pin[0].pull_dir ? PAD_PULL_UP : PAD_PULL_NONE;
	uint32_t cfg_pull_strength = pin[0].pull_strength;
	uint32_t current_level = pin[0].current_level;

	/* set pull strength */
	Pad_PullConfigValue(cfg_pin, cfg_pull_strength);

	/* set current level */
	switch (current_level) {
	case 0:
		Pad_SetDrivingCurrent(cfg_pin, PAD_DRIVING_CURRENT_8_8mA);
		break;

	case 1:
		Pad_SetDrivingCurrent(cfg_pin, PAD_DRIVING_CURRENT_12_18mA);
		break;

	case 2:
		Pad_SetDrivingCurrent(cfg_pin, PAD_DRIVING_CURRENT_16_28mA);
		break;

	case 3:
		Pad_SetDrivingCurrent(cfg_pin, PAD_DRIVING_CURRENT_16_28mA);
		break;

	default:
		break;
	}

	/* set pad and pinmux */
	if (cfg_fun == BEE_PWR_OFF) {
		Pad_Config(cfg_pin, PAD_SW_MODE, PAD_NOT_PWRON, cfg_pull, cfg_dir, cfg_drv);
	} else if (cfg_fun == BEE_SW_MODE) {
		Pad_Config(cfg_pin, PAD_SW_MODE, PAD_IS_PWRON, cfg_pull, cfg_dir, cfg_drv);
	} else if (cfg_fun < BEE_PINMUX_MAX) {
		Pad_Config(cfg_pin, PAD_PINMUX_MODE, PAD_IS_PWRON, cfg_pull, cfg_dir, cfg_drv);
		Pinmux_Config(cfg_pin, cfg_fun);
	}
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		pinctrl_configure_pin(&pins[i]);
	}

	return 0;
}
