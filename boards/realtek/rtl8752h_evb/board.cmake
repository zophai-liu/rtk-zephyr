# Copyright (c) 2024 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0
board_runner_args(jlink "--device=RTL8752H" "--speed=4000")
board_runner_args(pyocd "--target=RTL8752H" "--tool-opt=--pack=${ZEPHYR_REALTEK_ZEPHYR_PROJECT_MODULE_DIR}/support/${CONFIG_SOC_SERIES}/Realtek.RTL8752H_DFP.1.0.0.pack")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
