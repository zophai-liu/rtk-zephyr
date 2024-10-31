/*
 * Copyright(c) 2020, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_rtl8752h_flash_controller
#define SOC_NV_FLASH_NODE DT_INST(0, soc_nv_flash)

#define FLASH_WRITE_BLK_SZ DT_PROP(SOC_NV_FLASH_NODE, write_block_size)
#define FLASH_ERASE_BLK_SZ DT_PROP(SOC_NV_FLASH_NODE, erase_block_size)

#define FLASH_SIZE DT_REG_SIZE(SOC_NV_FLASH_NODE)
#define FLASH_ADDR DT_REG_ADDR(SOC_NV_FLASH_NODE)


#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>

#include <flash_nor_device.h>
#include <trace.h>

LOG_MODULE_REGISTER(flash_rtl8752h, CONFIG_FLASH_LOG_LEVEL);
struct flash_rtl8752h_data {
	struct k_sem mutex;
};

#ifdef CONFIG_FLASH_PAGE_LAYOUT
static const struct flash_pages_layout flash_pages_layout_rtl8752h[] = {
	{
		.pages_size = FLASH_ERASE_BLK_SZ,
		.pages_count = FLASH_SIZE / FLASH_ERASE_BLK_SZ
	}
};
static void
flash_rtl8752h_page_layout(const struct device *dev,
						const struct flash_pages_layout **layout,
						size_t *layout_size)
{
	*layout = flash_pages_layout_rtl8752h;

	/*
	 * For flash memories which have uniform page sizes, this routine
	 * returns an array of length 1, which specifies the page size and
	 * number of pages in the memory.
	 */
	*layout_size = 1;
}
#endif

static const struct flash_parameters flash_rtl8752h_parameters = {
	.write_block_size = FLASH_WRITE_BLK_SZ,
	.erase_value = 0xff,
};

static int flash_rtl8752h_read(const struct device *dev, off_t offset,
							   void *data, size_t len)
{
	if ((offset > FLASH_SIZE) ||
		((offset + len) > FLASH_SIZE)) {
		LOG_ERR("offset(:0x%lx) or offset+len(:0x%lx) out of flash boundary", (long)offset,
				(long)(offset + len));
		return -EINVAL;
	}

	if (len == 0U) {
		return 0;
	}

	flash_nor_read_locked(FLASH_ADDR + offset, (uint8_t *)data, len);

	return 0;
}

static int flash_rtl8752h_write(const struct device *dev, off_t offset,
								const void *data, size_t len)
{
	if ((offset > FLASH_SIZE) ||
		((offset + len) > FLASH_SIZE)) {
		LOG_ERR("offset(:0x%lx) or offset+len(:0x%lx) out of flash boundary", (long)offset,
				(long)(offset + len));
		return -EINVAL;
	}

	if (len == 0U) {
		return 0;
	}

	if (data >= (const void *)FLASH_ADDR) {
		char tmp[len];

		flash_nor_read_locked((uint32_t)data, (uint8_t *)tmp, len);
		flash_nor_write_locked(FLASH_ADDR + offset, (uint8_t *)tmp, len);
		return 0;
	}

	flash_nor_write_locked(FLASH_ADDR + offset, (uint8_t *)data, len);
	return 0;
}

static int flash_rtl8752h_erase(const struct device *dev, off_t offset, size_t size)
{
	if ((offset > FLASH_SIZE) ||
		((offset + size) > FLASH_SIZE)) {
		LOG_ERR("offset(:0x%lx) or offset+size(:0x%lx) is out of flash boundary",
			(long)offset, (long)(offset + size));
		return -EINVAL;
	}

	if ((offset % FLASH_ERASE_BLK_SZ) != 0) {
		LOG_ERR("offset 0x%lx: not on a page boundary", (long)offset);
		return -EINVAL;
	}

	if ((size % FLASH_ERASE_BLK_SZ) != 0) {
		LOG_ERR("size %zu: not multiple of a page size", size);
		return -EINVAL;
	}

	if (!size) {
		return 0;
	}

	uint32_t start_addr = FLASH_ADDR + offset;

	for (int i = 0; i < size / FLASH_ERASE_BLK_SZ; i++) {
		uint32_t key = arch_irq_lock();

		flash_nor_erase_locked(start_addr + i * FLASH_ERASE_BLK_SZ, FLASH_NOR_ERASE_SECTOR);
		arch_irq_unlock(key);
	}

	return 0;
}

static const struct flash_parameters *
flash_rtl8752h_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_rtl8752h_parameters;
}

static const struct flash_driver_api flash_rtl8752h_driver_api = {
	.read = flash_rtl8752h_read,
	.write = flash_rtl8752h_write,
	.erase = flash_rtl8752h_erase,
	.get_parameters = flash_rtl8752h_get_parameters,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_rtl8752h_page_layout,
#endif
};

#define GET_FLASH_BIT_MODE_STR(mode) \
	((mode) == FLASH_NOR_1_BIT_MODE ? "FLASH_NOR_1_BIT_MODE" : \
	(mode) == FLASH_NOR_2_BIT_MODE ? "FLASH_NOR_2_BIT_MODE" : \
	(mode) == FLASH_NOR_4_BIT_MODE ? "FLASH_NOR_4_BIT_MODE" : "Invalid mode")
static int flash_rtl8752h_init(const struct device *dev)
{
	/* ToDo */
	if (flash_nor_try_high_speed_mode(FLASH_NOR_IDX_SPIC0,
		CONFIG_SOC_FLASH_RTL8752H_BIT_MODE) == FLASH_NOR_RET_SUCCESS) {
		LOG_INF("Flash change to %s",
			GET_FLASH_BIT_MODE_STR(CONFIG_SOC_FLASH_RTL8752H_BIT_MODE)
		);
	}
	return 0;
}

static struct flash_rtl8752h_data flash_data;

DEVICE_DT_INST_DEFINE(0, flash_rtl8752h_init, NULL,
					  &flash_data, NULL, POST_KERNEL,
					  CONFIG_FLASH_INIT_PRIORITY, &flash_rtl8752h_driver_api);
