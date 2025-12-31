/*
 * Copyright (c) 2026 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_BEE_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_BEE_PINCTRL_H_

/*
 * The whole Bee pin configuration information is encoded in a 32-bit bitfield
 * organized as follows:
 *
 * - 0..10:  Pin number.
 * - 11..12: Pin pull configuration.
 * - 13:     Pin output drive configuration.
 * - 14:     Pin direction configuration.
 * - 15:     Pin pull strength.
 * - 16..31: Pin function.
 */

/** Position of the function field. */
#define BEE_FUN_POS   16U
/** Mask for the function field. */
#define BEE_FUN_MSK   0xFFFFU
/** Position of the direction field. */
#define BEE_DIR_POS   14U
/** Mask for the low direction field. */
#define BEE_DIR_MSK   0x1U
/** Position of the drive configuration field. */
#define BEE_DRIVE_POS 13U
/** Mask for the drive configuration field. */
#define BEE_DRIVE_MSK 0x1U
/** Position of the pull configuration field. */
#define BEE_PULL_POS  11U
/** Mask for the pull configuration field. */
#define BEE_PULL_MSK  0x3U
/** Position of the pin field. */
#define BEE_PIN_POS   0U
/** Mask for the pin field. */
#define BEE_PIN_MSK   0x7FFU

#define BEE_PSEL(fun, pin, dir, drive, pull)                                                       \
	(((((pin) & BEE_PIN_MSK) << BEE_PIN_POS) | (((BEE_##fun) & BEE_FUN_MSK) << BEE_FUN_POS)) | \
	 ((((BEE_##dir) & BEE_DIR_MSK) << BEE_DIR_POS) |                                           \
	  (((BEE_##drive) & BEE_DRIVE_MSK) << BEE_DRIVE_POS) |                                     \
	  (((BEE_##pull) & BEE_PULL_MSK) << BEE_PULL_POS)))

#define BEE_PSEL_DISCONNECTED(fun)                                                                 \
	(BEE_PIN_DISCONNECTED << BEE_PIN_POS | ((BEE_##fun & BEE_FUN_MSK) << BEE_FUN_POS))

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_BEE_PINCTRL_H_ */
