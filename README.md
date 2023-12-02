# XIAO BLE Sense (nRF52840) Battery Management Library

<p align="center">
   <br>
 <img src="https://img.shields.io/github/stars/tjoms99/xiao_sense_nrf52840_battery_lib?logo=github&color=yellow" alt="GitHub Repo stars">
 <img src="https://hits.seeyoufarm.com/api/count/incr/badge.svg?url=https%3A%2F%2Fgithub.com%2FTjoms99%2Fxiao_sense_nrf52840_battery_lib&count_bg=%2379C83D&title_bg=%23555555&icon=azurefunctions.svg&icon_color=%23E1E1E1&title=hits&edge_flat=false" alt="Hits">
</p>

## Overview
This library is designed to manage the battery charging functionality of the XIAO BLE Sense board. It supports the following features for a 3.7V LiPo battery:

- Reading battery voltage.
- Setting charging modes (fast or slow).
- Enabling or disabling charging.
- Calculating battery capacity in percentage.

The library is built on the Zephyr Real-Time Operating System (RTOS). 
For comprehensive details on Zephyr and how to get started, visit the 
[Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html).

## Programming and Debugging
### Adafruit nRF52 Bootloader
The XIAO BLE Sense is equipped with the Adafruit nRF52 Bootloader 5, facilitating the 
UF2 flashing process.

### UF2 Flashing Process

1. **Entering Bootloader Mode**: Connect the XIAO BLE Sense to your computer via USB. 
   Then, double-tap the reset button located to the left of the USB connector. This 
   action will enable the bootloader mode, and the device will appear as a mass storage 
   device named 'XIAO BLE' on your computer.

2. **Flashing the Firmware**:

   - Navigate to the '**build/zephyr/**' directory and locate the '**zephyr.uf2**' file. If you can not find the build folder, build or rebuild the zephyr project. 
   - Copy the **zephyr.uf2** file to the root directory of the XIAO BLE mass storage device using either the command line or your file manager.
3. **Automatic Reset and Application Launch**: After the UF2 file transfer is complete, 
   the XIAO BLE Sense will automatically reset and launch the new application.

For additional information on the flashing process and the XIAO BLE Sense board, refer to 
the [Zephyr Board Documentation for XIAO BLE](https://docs.zephyrproject.org/latest/boards/arm/xiao_ble/doc/index.html).
