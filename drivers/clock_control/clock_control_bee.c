/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_bee_cctl

#include <stdint.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>

#if defined(CONFIG_SOC_SERIES_RTL8752H)
#include <rtl876x_rcc.h>
#endif

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(clock_control_bee, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

struct clock_control_bee_config {
	uint32_t reg;
};

struct apb_cfg {
	uint32_t apbperiph;
	uint32_t apbperiph_clk;
};

#if defined(CONFIG_SOC_SERIES_RTL8752H)
static const struct apb_cfg bee_apb_table[] = {
	{APBPeriph_I2S0, APBPeriph_I2S0_CLOCK},
	{APBPeriph_I2S1, APBPeriph_I2S1_CLOCK},
	{APBPeriph_CODEC, APBPeriph_CODEC_CLOCK},
	{APBPeriph_GPIO, APBPeriph_GPIO_CLOCK},
	{APBPeriph_GDMA, APBPeriph_GDMA_CLOCK},
	{APBPeriph_TIMER, APBPeriph_TIMER_CLOCK},
	{APBPeriph_ENHTIMER, APBPeriph_ENHTIMER_CLOCK},
	{APBPeriph_UART2, APBPeriph_UART2_CLOCK},
	{APBPeriph_UART0, APBPeriph_UART0_CLOCK},
	{APBPeriph_FLASH, APBPeriph_FLASH_CLOCK},
	{APBPeriph_PKE, APBPeriph_PKE_CLOCK},
	{APBPeriph_SHA256, APBPeriph_SHA256_CLOCK},
	{APBPeriph_FLASH1, APBPeriph_FLASH1_CLOCK},
	{APBPeriph_FLH_SEC, APBPeriph_FLH_SEC_CLOCK},
	{APBPeriph_IR, APBPeriph_IR_CLOCK},
	{APBPeriph_SPI1, APBPeriph_SPI1_CLOCK},
	{APBPeriph_SPI0, APBPeriph_SPI0_CLOCK},
	{APBPeriph_UART1, APBPeriph_UART1_CLOCK},
	{APBPeriph_IF8080, APBPeriph_IF8080_CLOCK},
	{APBPeriph_ADC, APBPeriph_ADC_CLOCK},
	{APBPeriph_SPI2W, APBPeriph_SPI2W_CLOCK},
	{APBPeriph_MODEMRFCPI_CLOCK, APBPeriph_MODEMRFCPI_CLOCK},
	{APBPeriph_BLUEWIZ, APBPeriph_BLUEWIZ_CLOCK},
	{APBPeriph_ZIGBEE, APBPeriph_ZIGBEE_CLOCK},
	{APBPeriph_KEYSCAN, APBPeriph_KEYSCAN_CLOCK},
	{APBPeriph_QDEC, APBPeriph_QDEC_CLOCK},
	{APBPeriph_I2C1, APBPeriph_I2C1_CLOCK},
	{APBPeriph_I2C0, APBPeriph_I2C0_CLOCK},
};
#endif

static int clock_control_bee_on(const struct device *dev, clock_control_subsys_t sys)
{
	uint16_t id = *(uint16_t *)sys;

	RCC_PeriphClockCmd(bee_apb_table[id].apbperiph, bee_apb_table[id].apbperiph_clk, ENABLE);
	LOG_DBG("sys=%d, apbperiph=0x%x, apbperiph_clk=0x%x", id,
		   bee_apb_table[id].apbperiph, bee_apb_table[id].apbperiph_clk);
	return 0;
}

static int clock_control_bee_off(const struct device *dev, clock_control_subsys_t sys)
{
	uint16_t id = *(uint16_t *)sys;

	RCC_PeriphClockCmd(bee_apb_table[id].apbperiph, bee_apb_table[id].apbperiph_clk, DISABLE);

	LOG_DBG("sys=%d, apbperiph=%d, apbperiph_clk=%d", id,
		   bee_apb_table[id].apbperiph, bee_apb_table[id].apbperiph_clk);
	return 0;
}

static DEVICE_API(clock_control, clock_control_bee_api) = {
	.on = clock_control_bee_on,
	.off = clock_control_bee_off,
};

static const struct clock_control_bee_config config = {
	.reg = DT_INST_REG_ADDR(0),
};

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, &config, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &clock_control_bee_api);
