/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <clock_manager.h>
#include <system_rtl876x.h>
#include <rtl_boot_record.h>

#ifdef CONFIG_SYSTICK_USE_EXTERNAL_CLOCK
/* The system clock frequency of RTL8752H is fixed at 32,000 Hz. */
BUILD_ASSERT(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC == 32000);
#endif

static int rtl8752h_platform_init(void)
{
	rtl_boot_stage_record(START_PLATFORM_INIT);

	/* Enable cache */
	share_cache_ram();

	/* Set clk src */
	set_active_mode_clk_src();
	set_up_32k_clk_src();

	rtl_boot_stage_record(AON_BOOT_DONE);

	/*
	 * Enable Systck 32K clock, vendor register, enable bus clock,
	 * init trng, init ram power control.
	 */
	hal_setup_hardware();

	hal_setup_cpu();
	return 0;
}

static int rtl8752h_sysclock_update(void)
{
#ifdef CONFIG_SYSTICK_USE_EXTERNAL_CLOCK
	SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;
#endif /* CONFIG_SYSTICK_USE_EXTERNAL_CLOCK */
	return 0;
}

SYS_INIT(rtl8752h_platform_init, EARLY, 0);
SYS_INIT(rtl8752h_sysclock_update, PRE_KERNEL_2, 1);
