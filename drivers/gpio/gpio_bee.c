/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_bee_gpio

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <soc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/clock_control/bee_clock_control.h>
#include <zephyr/sys/util.h>
#include <zephyr/irq.h>

#include <zephyr/dt-bindings/gpio/realtek-bee-gpio.h>

#ifdef GPIO_INT_MASK
#undef GPIO_INT_MASK
#endif

#if defined(CONFIG_SOC_SERIES_RTL8752H)
#include <rtl876x_rcc.h>
#include <rtl876x_pinmux.h>
#include <rtl876x_gpio.h>
#endif

#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/logging/log.h>

#if defined(CONFIG_SOC_SERIES_RTL8752H)
#define BEE_GPIO_WriteBit(port, bit, val)            GPIO_WriteBit(bit, val)
#define BEE_GPIO_ReadOutputData(port)                GPIO_ReadOutputData()
#define BEE_GPIO_ReadOutputDataBit(port, bit)        GPIO_ReadOutputDataBit(bit)
#define BEE_GPIO_INTConfig(port, bit, val)           GPIO_INTConfig(bit, val)
#define BEE_GPIO_Init(port, val)                     GPIO_Init(val)
#define BEE_GPIO_MaskINTConfig(port, bit, val)       GPIO_MaskINTConfig(bit, val)
#define BEE_GPIO_ClearINTPendingBit(port, bit)       GPIO_ClearINTPendingBit(bit)
#define BEE_GPIO_SetBits(port, bit)                  GPIO_SetBits(bit)
#define BEE_GPIO_ResetBits(port, bit)                GPIO_ResetBits(bit)
#define BEE_GPIO_ReadInputData(port)                 GPIO_ReadInputData()
#define BEE_GPIO_Write(port, val)                    GPIO_Write(val)
#define BEE_Pad_SetControlMode(pad, mode)            Pad_ControlSelectValue(pad, mode)
#define BEE_Pad_SetOutputLevel(pad, val)             Pad_OutputControlValue(pad, val)
#endif

LOG_MODULE_REGISTER(gpio_bee, CONFIG_GPIO_LOG_LEVEL);

struct gpio_bee_irq_info {
	const struct device *irq_dev;
	uint8_t num_irq;
	struct gpio_irq_info {
		uint32_t irq;
		uint32_t priority;
	} gpio_irqs[];
};

struct gpio_bee_config {
	struct gpio_driver_config common;
	uint16_t clkid;
	uint8_t port_num;
	GPIO_TypeDef *port_base;
	struct gpio_bee_irq_info *irq_info;
};

struct gpio_bee_data {
	struct gpio_driver_data common;
	const struct device *dev;
	sys_slist_t cb;
	uint8_t pin_debounce_ms[32];
};

static int gpio_bee_gpio2pad(uint8_t port_num, uint32_t pin)
{
#if defined(CONFIG_SOC_SERIES_RTL8752H)
	if (pin <= 9) {
		return pin;
	} else if (pin <= 12) {
		return pin + 26;
	} else if (pin == 13) {
		return 32;
	} else if (pin <= 28) {
		return pin;
	} else if (pin == 29) {
#if BEE_USE_P4_1_AS_GPIO29
		return 33;
#else
		return 29;
#endif
	} else if (pin == 30) {
#if BEE_USE_P4_2_AS_GPIO30
		return 34;
#else
		return 30;
#endif
	} else if (pin == 31) {
		return 35;
	}
#endif

	return -EIO;
}

static int gpio_bee_pin_configure(const struct device *port, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_bee_config *config = port->config;
	struct gpio_bee_data *data = port->data;
	GPIO_TypeDef *port_base;
	uint8_t port_num = config->port_num;
	uint32_t gpio_bit = BIT(pin);
	int pad_pin = gpio_bee_gpio2pad(port_num, pin);
	uint32_t pull_config;
	GPIO_InitTypeDef gpio_init_struct;
	uint8_t debounce_ms =
		(flags & BEE_GPIO_INPUT_DEBOUNCE_MS_MASK) >> BEE_GPIO_INPUT_DEBOUNCE_MS_POS;
	int ret = 0;

	LOG_DBG("port=%s, pin=%d, flags=0x%x, line%d\n", port->name, pin, flags, __LINE__);

	port_base = config->port_base;

	__ASSERT(pad_pin >= 0, "gpio port or pin error");

	if (flags & GPIO_OPEN_SOURCE) {
		ret = -ENOTSUP;
		return ret;
	}

	if (flags == GPIO_DISCONNECTED) {
		Pinmux_Deinit(pad_pin);
		Pad_Config(pad_pin, PAD_SW_MODE, PAD_NOT_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE,
			   PAD_OUT_HIGH);
	} else {
		/* configure pad pull status */
		if (flags & GPIO_PULL_UP) {
			pull_config = PAD_PULL_UP;
		} else if (flags & GPIO_PULL_DOWN) {
			pull_config = PAD_PULL_DOWN;
		} else {
			pull_config = PAD_PULL_NONE;
		}

		/* configure gpio */
		GPIO_StructInit(&gpio_init_struct);

		if (debounce_ms) {
#if defined(CONFIG_SOC_SERIES_RTL8752H)
			gpio_init_struct.GPIO_DebounceTime = debounce_ms;
#endif
			gpio_init_struct.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_ENABLE;
			data->pin_debounce_ms[pin] = debounce_ms;
		} else {
			gpio_init_struct.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_DISABLE;
			data->pin_debounce_ms[pin] = 0;
		}

		gpio_init_struct.GPIO_Pin = gpio_bit;
		gpio_init_struct.GPIO_Mode = flags & GPIO_OUTPUT ? GPIO_Mode_OUT : GPIO_Mode_IN;
		gpio_init_struct.GPIO_ITCmd = flags & GPIO_INT_ENABLE ? ENABLE : DISABLE;
		gpio_init_struct.GPIO_ITTrigger = flags & GPIO_INT_LEVELS_LOGICAL
							  ? GPIO_INT_Trigger_LEVEL
							  : GPIO_INT_Trigger_EDGE;
		gpio_init_struct.GPIO_ITPolarity = flags & GPIO_INT_LOW_0
							   ? GPIO_INT_POLARITY_ACTIVE_LOW
							   : GPIO_INT_POLARITY_ACTIVE_HIGH;
		Pad_Config(pad_pin, PAD_PINMUX_MODE, PAD_IS_PWRON, pull_config,
			   flags & GPIO_OUTPUT ? PAD_OUT_ENABLE : PAD_OUT_DISABLE,
			   flags & GPIO_OUTPUT_INIT_HIGH ? PAD_OUT_HIGH : PAD_OUT_LOW);
		Pinmux_Config(pad_pin, DWGPIO);

		switch (flags & (GPIO_OUTPUT | GPIO_OUTPUT_INIT_HIGH | GPIO_OUTPUT_INIT_LOW)) {
		case (GPIO_OUTPUT_HIGH):
			BEE_GPIO_WriteBit(port_base, gpio_bit, 1);
			break;
		case (GPIO_OUTPUT_LOW):
			BEE_GPIO_WriteBit(port_base, gpio_bit, 0);
			break;
		default:
			break;
		}

		/* to avoid trigger gpio interrupt */
		if (debounce_ms && (flags & GPIO_INT_ENABLE)) {
			BEE_GPIO_INTConfig(port_base, gpio_bit, DISABLE);
			BEE_GPIO_Init(port_base, &gpio_init_struct);
			BEE_GPIO_MaskINTConfig(port_base, gpio_bit, ENABLE);
			BEE_GPIO_INTConfig(port_base, gpio_bit, ENABLE);
			k_busy_wait(data->pin_debounce_ms[pin] * 2 * 1000);
			BEE_GPIO_ClearINTPendingBit(port_base, gpio_bit);
			BEE_GPIO_MaskINTConfig(port_base, gpio_bit, DISABLE);
		} else {
			BEE_GPIO_Init(port_base, &gpio_init_struct);
		}
	}

	return 0;
}

static int gpio_bee_port_get_raw(const struct device *port, gpio_port_value_t *value)
{
	const struct gpio_bee_config *config = port->config;
	GPIO_TypeDef *port_base;

	port_base = config->port_base;

	*value = BEE_GPIO_ReadInputData(port_base);

	return 0;
}

static int gpio_bee_port_set_masked_raw(const struct device *port, gpio_port_pins_t mask,
					gpio_port_value_t value)
{
	const struct gpio_bee_config *config = port->config;
	struct gpio_bee_data *data;
	GPIO_TypeDef *port_base;

	data = port->data;
	port_base = config->port_base;

	gpio_port_pins_t pins_value = BEE_GPIO_ReadInputData(port_base);

	pins_value = (pins_value & ~mask) | (mask & value);
	BEE_GPIO_Write(port_base, pins_value);

	return 0;
}

static int gpio_bee_port_set_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_bee_config *config = port->config;
	struct gpio_bee_data *data;
	GPIO_TypeDef *port_base;

	data = port->data;
	port_base = config->port_base;

	BEE_GPIO_SetBits(port_base, pins);

	return 0;
}

static int gpio_bee_port_clear_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_bee_config *config = port->config;
	struct gpio_bee_data *data;
	GPIO_TypeDef *port_base;

	data = port->data;
	port_base = config->port_base;

	BEE_GPIO_ResetBits(port_base, pins);

	return 0;
}

static int gpio_bee_port_toggle_bits(const struct device *port, gpio_port_pins_t pins)
{
	const struct gpio_bee_config *config = port->config;
	struct gpio_bee_data *data;
	GPIO_TypeDef *port_base;

	data = port->data;
	port_base = config->port_base;

	uint32_t pins_value = BEE_GPIO_ReadInputData(port_base);

	pins_value = (pins_value | pins) & ~(pins_value & pins);
	BEE_GPIO_Write(port_base, pins_value);
	LOG_DBG("port=%s, pin=0x%x, pins_value=0x%x, line%d\n", port->name, pins, pins_value,
		__LINE__);

	return 0;
}

static int gpio_bee_pin_interrupt_configure(const struct device *port, gpio_pin_t pin,
					    enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	const struct gpio_bee_config *config = port->config;
	struct gpio_bee_data *data = port->data;
	GPIO_TypeDef *port_base;
	uint32_t gpio_bit = BIT(pin);
	GPIO_InitTypeDef gpio_init_struct;

	port_base = config->port_base;

	LOG_DBG("port=%s, pin=%d, mode=0x%x, trig=0x%x, line%d\n", port->name, pin, mode, trig,
		__LINE__);

#ifdef CONFIG_GPIO_ENABLE_DISABLE_INTERRUPT
	if (mode == GPIO_INT_MODE_DISABLE_ONLY) {
		BEE_GPIO_MaskINTConfig(port_base, gpio_bit, ENABLE);
		BEE_GPIO_INTConfig(port_base, gpio_bit, DISABLE);
		return 0;
	} else if (mode == GPIO_INT_MODE_ENABLE_ONLY) {
		BEE_GPIO_INTConfig(port_base, gpio_bit, ENABLE);
		BEE_GPIO_MaskINTConfig(port_base, gpio_bit, DISABLE);
		return 0;
	}
#endif /* CONFIG_GPIO_ENABLE_DISABLE_INTERRUPT */

	BEE_GPIO_INTConfig(port_base, gpio_bit, DISABLE);

	GPIO_StructInit(&gpio_init_struct);

	gpio_init_struct.GPIO_Pin = gpio_bit;
	gpio_init_struct.GPIO_Mode = GPIO_Mode_IN;
	if (data->pin_debounce_ms[pin]) {
#if defined(CONFIG_SOC_SERIES_RTL8752H)
		gpio_init_struct.GPIO_DebounceTime = data->pin_debounce_ms[pin];
#endif
		gpio_init_struct.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_ENABLE;
	} else {
		gpio_init_struct.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_DISABLE;
	}

	if (mode == GPIO_INT_MODE_DISABLED) {
		return 0;
	} else if (mode == GPIO_INT_MODE_EDGE) {
		gpio_init_struct.GPIO_ITCmd = ENABLE;
		gpio_init_struct.GPIO_ITTrigger = GPIO_INT_Trigger_EDGE;
	} else if (mode == GPIO_INT_MODE_LEVEL) {
		gpio_init_struct.GPIO_ITCmd = ENABLE;
		gpio_init_struct.GPIO_ITTrigger = GPIO_INT_Trigger_LEVEL;
	}

	switch (trig) {
	case GPIO_INT_TRIG_LOW:
		gpio_init_struct.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_LOW;
		break;
	case GPIO_INT_TRIG_HIGH:
		gpio_init_struct.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_HIGH;
		break;
	case GPIO_INT_TRIG_BOTH:
#if CONFIG_BEE_GPIO_SUPPORT_BOTH_EDGE
		gpio_init_struct.GPIO_ITTrigger = GPIO_INT_BOTH_EDGE;
		break;
#endif
	default:
		return -ENOTSUP;
	}

	BEE_GPIO_Init(port_base, &gpio_init_struct);
	BEE_GPIO_MaskINTConfig(port_base, gpio_bit, ENABLE);
	BEE_GPIO_INTConfig(port_base, gpio_bit, ENABLE);

	/* to avoid trigger gpio interrupt */
	if (data->pin_debounce_ms[pin]) {
		k_busy_wait(data->pin_debounce_ms[pin] * 2 * 1000);
	}

	BEE_GPIO_ClearINTPendingBit(port_base, gpio_bit);
	BEE_GPIO_MaskINTConfig(port_base, gpio_bit, DISABLE);

	return 0;
}

static int gpio_bee_manage_callback(const struct device *port, struct gpio_callback *cb, bool set)
{
	struct gpio_bee_data *port_data = port->data;

	return gpio_manage_callback(&port_data->cb, cb, set);
}

static uint32_t gpio_bee_get_pending_int(const struct device *dev)
{
	const struct gpio_bee_config *config = dev->config;
	GPIO_TypeDef *port_base = config->port_base;

	return GPIO_GetPortIntStatus(port_base);
}

#ifdef CONFIG_GPIO_GET_DIRECTION
int gpio_bee_port_get_direction(const struct device *port, gpio_port_pins_t map,
				gpio_port_pins_t *inputs, gpio_port_pins_t *outputs)
{
	const struct gpio_bee_config *config = port->config;
	GPIO_TypeDef *port_base = config->port_base;
	gpio_port_pins_t gpio_dir_status = GPIO_GetPortDirection(port_base);

	if (inputs != NULL) {
		*inputs = gpio_dir_status;
	}

	if (outputs != NULL) {
		*outputs = ~gpio_dir_status;
	}

	return 0;
}
#endif

static const struct gpio_driver_api gpio_bee_driver_api = {
	.pin_configure = gpio_bee_pin_configure,
	.port_get_raw = gpio_bee_port_get_raw,
	.port_set_masked_raw = gpio_bee_port_set_masked_raw,
	.port_set_bits_raw = gpio_bee_port_set_bits_raw,
	.port_clear_bits_raw = gpio_bee_port_clear_bits_raw,
	.port_toggle_bits = gpio_bee_port_toggle_bits,
	.pin_interrupt_configure = gpio_bee_pin_interrupt_configure,
	.manage_callback = gpio_bee_manage_callback,
	.get_pending_int = gpio_bee_get_pending_int,
#ifdef CONFIG_GPIO_GET_DIRECTION
	.port_get_direction = gpio_bee_port_get_direction,
#endif
};

static void gpio_bee_isr(void *arg)
{
	const struct device *dev = (struct device *)arg;
	const struct gpio_bee_config *config = dev->config;
	struct gpio_bee_data *data = dev->data;
	GPIO_TypeDef *port_base = config->port_base;
	const struct device *port = dev;
	uint32_t pins = GPIO_GetPortIntStatus(port_base);

	gpio_fire_callbacks(&data->cb, port, pins);

	for (uint32_t i = 0; i < 32; i++) {
		if (BIT(i) & pins) {
			BEE_GPIO_ClearINTPendingBit(port_base, BIT(i) & pins);
		}
	}
}

static int gpio_bee_init(const struct device *dev)
{
	struct gpio_bee_data *data = dev->data;
	const struct gpio_bee_config *config = dev->config;
	int ret = 0;

	(void)clock_control_on(BEE_CLOCK_CONTROLLER, (clock_control_subsys_t)&config->clkid);

	for (uint8_t i = 0; i < config->irq_info->num_irq; ++i) {
		irq_connect_dynamic(config->irq_info->gpio_irqs[i].irq,
				    config->irq_info->gpio_irqs[i].priority,
				    (const void *)gpio_bee_isr, dev, 0);
		irq_enable(config->irq_info->gpio_irqs[i].irq);
	}

	data->dev = dev;
	memset(data->pin_debounce_ms, 0, sizeof(data->pin_debounce_ms));

	return ret;
}

#define GPIO_BEE_SET_GPIO_IRQ_INFO(irq_idx, index)                                                 \
	{                                                                                          \
		.irq = DT_INST_IRQ_BY_IDX(index, irq_idx, irq),                                    \
		.priority = DT_INST_IRQ_BY_IDX(index, irq_idx, priority),                          \
	}

#define GPIO_BEE_SET_IRQ_INFO(index)                                                               \
	static struct gpio_bee_irq_info gpio_bee_irq_info##index = {                               \
		.gpio_irqs = {LISTIFY(DT_NUM_IRQS(DT_DRV_INST(index)), GPIO_BEE_SET_GPIO_IRQ_INFO, \
				      (,), index)},                                               \
		.num_irq = DT_NUM_IRQS(DT_DRV_INST(index))};

#define GPIO_BEE_GET_IRQ_INFO(index) .irq_info = &gpio_bee_irq_info##index,

#define GPIO_BEE_DEVICE_INIT(index)                                                                \
	GPIO_BEE_SET_IRQ_INFO(index)                                                               \
	static const struct gpio_bee_config gpio_bee_port##index##_cfg = {                         \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(index),           \
			},                                                                         \
		.port_num = DT_INST_PROP(index, port),                                             \
		.port_base = (GPIO_TypeDef *)DT_INST_REG_ADDR(index),                              \
		.clkid = DT_INST_CLOCKS_CELL(index, id),                                           \
		GPIO_BEE_GET_IRQ_INFO(index)};                                                     \
                                                                                                   \
	static struct gpio_bee_data gpio_bee_port##index##_data;     \
	DEVICE_DT_INST_DEFINE(index, gpio_bee_init, NULL,                  \
			      &gpio_bee_port##index##_data, &gpio_bee_port##index##_cfg,           \
			      POST_KERNEL, CONFIG_GPIO_INIT_PRIORITY, &gpio_bee_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_BEE_DEVICE_INIT)
