/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/linker/linker-defs.h>
#include <kernel_internal.h>
#include <zephyr/arch/common/init.h>
#include <soc.h>

#include "mem_config.h"
#include "system_init_ns.h"
#include "rom_ns_cb.h"
#include "utils.h"
#include "sys_reset.h"

extern char __extram_data_start[];
extern char __extram_data_end[];
extern char __extram_data_load_start[];
extern char __extram_bss_start[];
extern char __extram_bss_end[];

static void rtl87x2g_extra_ram_init(void)
{
	arch_early_memcpy(&__extram_data_start, &__extram_data_load_start,
			__extram_data_end - __extram_data_start);
	arch_early_memcpy(__extram_bss_start, 0, __extram_bss_end - __extram_bss_start);
}

static int rtl87x2g_platform_init(void)
{
	rtl87x2g_extra_ram_init();
	/*
	 * RTL87X2G reserves a RAM region for the vector table, referred to as the RamVectorTable.
	 * Steps to initialize the vector table in RAM:
	 * 1. Set the SCB->VTOR register to point to the start address of the RamVectorTable.
	 * 2. Copy Zephyr's vector table to the RamVectorTable.
	 */
	size_t vector_size = (size_t)_vector_end - (size_t)_vector_start;
#ifdef CONFIG_TRUSTED_EXECUTION_NONSECURE
	/* tz enabled */
	SCB->VTOR = (uint32_t)NS_RAM_VECTOR_ADDR;
	(void)memcpy((void *)NS_RAM_VECTOR_ADDR, _vector_start, vector_size);
#else
	/* tz disabled */
	SCB->VTOR = (uint32_t)S_RAM_VECTOR_ADDR;
	(void)memcpy((void *)S_RAM_VECTOR_ADDR, _vector_start, vector_size);
#endif

	/* TZ enabled: for "Non-secure function call".
	 * Init non-secure function pointer that will be called by secure side using
	 * cmse_nsfptr_create().
	 * Example: RTK FLASH APIs in secure side would call os_lock() which is a non-secure
	 function.
	 * Sample code: nonsecure_os_lock = (NS_UINT32_PATCH_FUNC)cmse_nsfptr_create(func);
	 * Link: https://developer.arm.com/documentation/100720/0200/CMSE-support

	 * TZ disabled: no special process, just a common function pointer assignment.
	 * Sample code: secure_os_lock = (UINT32_PATCH_FUNC)((uint32_t)func | 0x1);
	 */
	secure_os_func_ptr_init();

	/* Function same as secure_os_func_ptr_init. But the non-secure function pointer
	 * is write_info_to_flash_before_reset that is used in WDG_SystemReset_Dump
	 * (secure function).
	 */
	secure_platform_func_ptr_init();

	/* Configure Memory Attritube through MPU.
	 * Refer to boot_cfg.common.mpu_region[8].
	 */
	mpu_setup();

	/* RXI300 init*/
	hal_setup_hardware();

	/* dwt init, mpu setup(again), init FPU */
	hal_setup_cpu();

#ifdef CONFIG_TRUSTED_EXECUTION_NONSECURE
	/* Set certain interrupts to be generated in NS mode.*/
	setup_non_secure_nvic();
#endif

	return 0;
}

static int rtl87x2g_update_systick_config(void)
{
	/* rtl87x2g's cortex-m systick timer is using external clock source
	 * instead of cpu clock as referance.
	 * The priority of systick interrupt is lowest for rtl87x2g SoCs.
	 */
	NVIC_SetPriority(SysTick_IRQn, 0xff);
	SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;

	return 0;
}

#ifdef CONFIG_ARCH_HAS_CUSTOM_BUSY_WAIT
void arch_busy_wait(uint32_t usec_to_wait)
{
	platform_delay_us(usec_to_wait);
}
#endif

/* Overrides the weak ARM implementation */
void sys_arch_reboot(int type)
{
	WDG_SystemReset(0, type);
}

SYS_INIT(rtl87x2g_platform_init, EARLY, 0);
SYS_INIT(rtl87x2g_update_systick_config, PRE_KERNEL_2, 1);
