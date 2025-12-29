/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <clock_manager.h>
#include <system_rtl876x.h>

static int rtl8752h_platform_init(void)
{
	si_flow_data_init();
	ft_paras_apply();
	/* enable cache */
	extern void share_cache_ram(void);
	share_cache_ram();

	/*Set the active mode clk src.*/
	set_active_mode_clk_src();
	set_up_32k_clk_src();
	work_around_32k_power_glitch_after_restart();

	/* Enable Systck 32K clock, vendor register, enable bus clock,
	 * init trng, init ram power control.
	 */
	hal_setup_hardware();

	hal_setup_cpu();

	return 0;
}

static int rtl8752h_sysclock_update(void)
{
#ifdef CONFIG_SYSTICK_USE_EXTERNAL_CLOCK
#if (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC != 32000)
#error "CONFIG_SYSTICK_USE_EXTERNAL_CLOCK does not match CONFIG_SYS_CLOCK_TICKS_PER_SEC"
#endif
	SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;
#else
#if (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC != 40000000)
#error "CPU Clock Rate does not match CONFIG_SYS_CLOCK_TICKS_PER_SEC"
#endif
#endif /* CONFIG_SYSTICK_USE_EXTERNAL_CLOCK */
	return 0;
}

SYS_INIT(rtl8752h_platform_init, EARLY, 0);
SYS_INIT(rtl8752h_sysclock_update, PRE_KERNEL_2, 1);
