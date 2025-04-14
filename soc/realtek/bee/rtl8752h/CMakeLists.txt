# Copyright (c) 2024 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0
zephyr_library_sources_ifdef(CONFIG_REALTEK_IMAGE_HEADER image_header.c)
zephyr_linker_sources(ROM_START SORT_KEY 0x0header realtek_image_header.ld)
zephyr_linker_sources(SECTIONS trace_section.ld)
zephyr_library_sources(soc.c)
zephyr_include_directories(.)
set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")
if (CONFIG_CODE_DATA_RELOCATION)

# Place some bss data on BufferRAM to save DataRAM for APP
# zephyr_code_relocate(FILES ${ZEPHYR_BASE}/subsys/logging/log_core.c LOCATION RAM2_BSS)
zephyr_code_relocate(FILES ${ZEPHYR_BASE}/subsys/shell/backends/shell_uart.c LOCATION RAM2_BSS)

if (CONFIG_PM)
# relocate: z_arm_nmi
zephyr_code_relocate(FILES ${ZEPHYR_BASE}/arch/${ARCH}/core/nmi.c LOCATION RAM)

zephyr_library_sources(power.c)
endif()
endif()

zephyr_sources_ifdef(CONFIG_ARM_MPU mpu_regions.c)
