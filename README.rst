.. _xiao_sense_nrf52840_battery_lib:

XIAO BLE Sense (nRF52840) battery library
###########

Overview
********

A library that can be used with the XIAO BLE Sense board to read the battery voltage, set fast or slow charging,
enable/disable charging,
and get the capacity of a 3.7V LiPo battery in percent.
The library uses the zephyr RTOS. For more information about Zephyr, check out the getting started guide: https://docs.zephyrproject.org/latest/develop/getting_started/index.html.

Programming and Debugging
********************

The XIAO BLE ships with the Adafruit nRF52 Bootloader 5 which supports flashing using UF2.

UF2 Flashing
=============
To enter the bootloader, connect the USB port of the XIAO BLE to your computer and double-tap the reset button to the left of the USB connector.
A mass storage device named XIAO BLE should appear on the computer. Using the command line, or your file manager, copy the build/zephyr/zephyr.uf2 file from your build to the base of the XIAO BLE mass storage device.
The XIAO BLE will automatically reset and launch the newly flashed application. For more information, check out the overview here:  https://docs.zephyrproject.org/latest/boards/arm/xiao_ble/doc/index.html.
