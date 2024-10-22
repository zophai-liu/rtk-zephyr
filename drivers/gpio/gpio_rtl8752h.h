/*
 * Copyright(c) 2020, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_GPIO_GPIO_RTL8752H_H_
#define ZEPHYR_DRIVERS_GPIO_GPIO_RTL8752H_H_

/**
 * @file header for RTL8752H GPIO
 */

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/gpio.h>
#include <rtl876x_gpio.h>

/* GPIO buses definitions */

struct gpio_rtl8752h_irq_info {
	const struct device *irq_dev;
	uint8_t num_irq;
	struct gpio_irq_info {
		uint32_t irq;
		uint32_t priority;
	} gpio_irqs[];
};

/**
 * @brief configuration of GPIO device
 */
struct gpio_rtl8752h_config {
	struct gpio_driver_config common;
	uint16_t clkid;
	GPIO_TypeDef *port_base;
	struct gpio_rtl8752h_irq_info *irq_info;
};

#ifdef CONFIG_PM_DEVICE
struct pm_pad_node {
	uint8_t pad_num;
	uint8_t next_gpio_num;
};

struct pm_pad_node_list {
	struct pm_pad_node *output_head;
	struct pm_pad_node *wakeup_head;
	struct pm_pad_node *array;
};

#endif

/**
 * @brief driver data
 */
struct gpio_rtl8752h_data {
	struct gpio_driver_data common;
	const struct device *dev;
	sys_slist_t cb;
	uint8_t pin_debounce_ms[32];
#ifdef CONFIG_PM_DEVICE
	GPIOStoreReg_Typedef store_buf;
	struct pm_pad_node_list list;
#endif
};

/**
 * @brief helper for configuration of GPIO pin
 *
 * @param dev GPIO port device pointer
 * @param pin IO pin
 * @param conf GPIO mode
 * @param func Pin function
 *
 * @return 0 on success, negative errno code on failure
 */
int gpio_rtl8752h_configure(const struct device *dev, int pin, int conf, int func);

#endif /* ZEPHYR_DRIVERS_GPIO_GPIO_RTL8752H_H_ */
