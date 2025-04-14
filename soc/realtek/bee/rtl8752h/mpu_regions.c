/*
 * Copyright (c) 2017 Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <zephyr/init.h>
#include <zephyr/sys/slist.h>
#include <zephyr/arch/arm/mpu/arm_mpu.h>
#include <zephyr/arch/arm/cortex_m/arm_mpu_mem_cfg.h>

#include <mem_config.h>

#define REALTEK_MPU_REGION_CONFIGURATION_GET 1

#define REGION_DATA_RAM_ATTR(size)                                                                 \
	{(NORMAL_OUTER_INNER_WRITE_THROUGH_SHAREABLE | size | P_RW_U_RW_Msk)}
#define REGION_ROM_ATTR(size) {(NORMAL_OUTER_INNER_WRITE_THROUGH_NON_SHAREABLE | size | RO_Msk)}
/* Shareable Device, Privileged Read Write, Unprivileged Read Write */
// #define REGION_PERIPHERAL_ATTR(size) {(DEVICE_SHAREABLE | size | P_RW_U_RW_Msk)}
#define REGION_PERIPHERAL_ATTR(size) {(MPU_RASR_B_Msk | size | P_RW_U_RW_Msk)}

static const struct arm_mpu_region mpu_regions[] = {
	/* Region 0: ROM(first 256KB) read only */
	MPU_REGION_ENTRY("ROM_0", 0x0, REGION_ROM_ATTR(REGION_256K)),

	/* Region 1: ROM(next 96KB) read only */
	MPU_REGION_ENTRY("ROM_1", 0x40000,
			 // disable sub-region 6, 7
			 REGION_ROM_ATTR(REGION_128K | 0xC0 << MPU_RASR_SRD_Pos)),

	/* Region 2: Flash, with specific sub-regions disabled */
	MPU_REGION_ENTRY("FLASH", 0x0,
			 /* Flash address, disable sub-region 0, 2, 4, 5, 6, 7 */
			 REGION_FLASH_ATTR(REGION_64M | 0xF5 << MPU_RASR_SRD_Pos)),

	/* Region 3: Data RAM (first 96KB) */
	MPU_REGION_ENTRY("DATA_RAM_0", DATA_RAM_START_ADDR,
			 REGION_DATA_RAM_ATTR(REGION_128K | 0xC0 << MPU_RASR_SRD_Pos)),

	/* Region 4: Data RAM (next 16KB) */
	MPU_REGION_ENTRY("DATA_RAM_1", DATA_RAM_START_ADDR + 0x18000,
			 // All cache used as cache, not as RAM. disable sub-region 4, 5, 6, 7
			 REGION_DATA_RAM_ATTR(REGION_16K | 0xF0 << MPU_RASR_SRD_Pos)),

	/* Region 5: Buffer RAM (16KB) */
	MPU_REGION_ENTRY("BUFFER_RAM", BUFFER_RAM_START_ADDR, REGION_DATA_RAM_ATTR(REGION_16K)),

	/* Region 6: Peripheral */
	MPU_REGION_ENTRY("PERIPHERAL", 0x40000000, REGION_PERIPHERAL_ATTR(REGION_512M)),
};

const struct arm_mpu_config mpu_config = {
	.num_regions = ARRAY_SIZE(mpu_regions),
	.mpu_regions = mpu_regions,
};

#if REALTEK_MPU_REGION_CONFIGURATION_GET
#include <trace.h>
static inline uint8_t get_num_regions(void)
{
	uint32_t type = MPU->TYPE;

	type = (type & MPU_TYPE_DREGION_Msk) >> MPU_TYPE_DREGION_Pos;

	return (uint8_t)type;
}
static int mpu_region_configuration_get()
{
	static bool mpu_updated;

	for (int index = 0; index < get_num_regions(); index++) {
		MPU->RNR = index;
		if (mpu_updated) {
			DBG_DIRECT("MPU Region(has updated)[%d] 0x%08x 0x%08x", MPU->RNR, MPU->RBAR,
				   MPU->RASR);
		} else {
			DBG_DIRECT("MPU Region(not updated)[%d] 0x%08x 0x%08x", MPU->RNR, MPU->RBAR,
				   MPU->RASR);
		}
	}
	mpu_updated = true;
	return 0;
}

SYS_INIT_NAMED(mpu_region_configuration_get_rom, mpu_region_configuration_get, EARLY, 1);
SYS_INIT_NAMED(mpu_region_configuration_get_curr, mpu_region_configuration_get, PRE_KERNEL_1, 1);

#endif /* REALTEK_MPU_REGION_CONFIGURATION_GET */
