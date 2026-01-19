.. zephyr:board:: rtl8752h_evb

Overview
********

RTL8752H supports Bluetooth 5.4 + 2.4GHz + IEEE 802.15.4(Thread/Zigbee) with ultra low power consumption. Equipped with 40MHz ARM Cortex-M0+ processor, large size Flash/SRAM, and flexible GPlOs, widely used in remote controller, Mesh smart home, small UI HMI device, low-cost wearable device, smart health & medical, smart meters, smart lock, electronic shelf label, and other products.
For more information, check `RTL8752H Introduction`_.

Hardware
********

RTL8752H Series Features
========================

The features include the following:

- ARM Cortex-M0+ maximum frequency 40MHz
- 120KB SRAM totally
- 512KB/1024KB Flash (depends on part number)
- Bluetooth low energy and 802.15.4
- Supports Secure Boot
- Peripheral Interface:

  - Flexible GPIO design
  - Hardware Keyscan and Quad-decoder
  - Embedded IR transceiver and receiver
  - Real-Time Counters (RTC)
  - SPI master/slave ×2, Timers ×8, I2C ×2, PWM ×8, UART ×2
  - I8080/QSPI
  - 400ksps, 12bits, 6 channels AUXADC
  - I2S/PCM interface
  - Internal 32K RCOSC to keep BLE link
  - Embedded PGA and audio ADC
  - AES-128/192/256 encryption/decryption engine and TRNG

- RF Performance:
  - Tx Power: 0/4/7.5dBm adjustable (refer to datasheet for details)
  - BLE Rx Sensitivity: -97dBm@1M
  - 15.4 Rx Sensitivity: -102dBm@250 kbps O-QPSK DSSS
  - Fast AGC control to improve receiving dynamic range

Board Features
==============

RTL8752H evaluation board works along with an interchangeable daughterboard that houses
a RTL8752H series SoC.

- RTL8752HJL/RTL8752HMF Daughter Board
- RTL8752HJF/RTL8752HKF Daughter Board

More information about the board can be found at `RTL8752H Evaluation Board Guide`_.

.. zephyr:board-supported-hw::

Prerequisites
*************

Before run a Zephyr application on the RTL8752H board, some essential patch images provided by Realtek must be programmed into the device.

`RTL8752H EVB Hardware Connection and Download Guide`_ provides a structured approach to understanding these images, wiring for
download mode, and step-by-step instructions for flashing them.

Programming and Debugging
*************************

.. zephyr:board-supported-runners::

Flashing
========

A UART‑to‑USB(FT232RL) is integrated to RTL8752H evaluation board for ready‑to‑use programming and logging. Follow the instructions in the `Realtek Bee Flash Programmer (MPCli) Host Tools`_ page to configure MPCli.

1. Connect to USB Port CON1 with an USB cable.
2. Connect P3_0 to RX and P3_1 to TX using jumpers.
3. Pull P0_3 (LOG) low with a Dupont wire.
4. Reset the hardware or power on again to enter download mode.

.. code-block:: console

   $ west flash --bee-port /dev/ttyX

After the application is downloaded successfully, restore M0_3 (LOG) to floating state. Reset the hardware or power on again, and the application will run.

Alternatively, JLink can also be used to flash the board using
the ``--runner`` (or ``-r``) option. Follow the instructions in the `J-Link Setup Guide`_ page to configure JLink.

.. code-block:: console

   $ west flash -r jlink

Logging
=======

By default, the UART2 is used for logging.

+-------+--------------+
| PIN # | Signal Name  |
+=======+==============+
| P3_0     | UART2 TX  |
+-------+--------------+
| P3_1     | UART2 RX  |
+-------+--------------+

#. Connect the UART:

    - Connect to USB Port CON1 with an USB cable.
    - Connect P3_0 to RX and P3_1 to TX using jumpers.

#. Open a serial communication tool that you are familiar with:

    - Set the baud rate to 2000000.

#. Press the reset button:

    - You should see "Hello World! rtl8752h_evb/rtl8762hkf" in your terminal.

Debugging
*********

Before debugging, make sure the JLink has already been configured, following the instructions in the `J-Link Setup Guide`_ page. Then, you can debug an application in the usual way.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: rtl8752h_evb/rtl8762hkf
   :maybe-skip-config:
   :goals: debug

References
**********

.. target-notes::

.. _RTL8752H Introduction:
    https://www.realmcu.com/en/Home/Products/RTL8752H-Series

.. _RTL8752H Evaluation Board Guide:
    https://docs.realmcu.com/sdk/rtl8752h/common/en/latest/evb_guide/text_en/README.html

.. _Realtek Bee Flash Programmer (MPCli) Host Tools:
    https://docs.zephyrproject.org/latest/develop/flash_debug/host-tools.html#Realtek-Bee-Flash-Programmer-(MPCli)-Host-Tools

.. _RTL8752H Documentation:
    https://docs.realmcu.com/sdk/rtl8752h/common/en/latest/overview/text_en

.. _RTL8752H EVB Interfaces Distribution:
    https://docs.realmcu.com/sdk/rtl8752h/common/en/latest/evb_guide/text_en/README.html#interface-distribution

.. _J-Link Setup Guide:
    https://github.com/rtkconnectivity/realtek-zephyr-project/wiki/J%E2%80%90Link-Setup-Guide

.. _RTL8752H EVB Hardware Connection and Download Guide:
    https://github.com/rtkconnectivity/realtek-zephyr-project/wiki/%5BRTL8752H-EVB%5D-Hardware-Connection-and-Download-Guide
