# XIAO BLE Sense (nRF52840) Battery Management Library

<p align="center">
   <br>
 <img src="https://img.shields.io/github/stars/tjoms99/xiao_sense_nrf52840_battery_lib?logo=github&color=yellow" alt="GitHub Repo stars">
 <img src="https://hits.seeyoufarm.com/api/count/incr/badge.svg?url=https%3A%2F%2Fgithub.com%2FTjoms99%2Fxiao_sense_nrf52840_battery_lib&count_bg=%2379C83D&title_bg=%23555555&icon=azurefunctions.svg&icon_color=%23E1E1E1&title=hits&edge_flat=false" alt="Hits">
</p>

## Overview

This library is designed to manage the battery charging functionality of the XIAO BLE and XIAO BLE Sense board. It supports the following features for a 3.7V LiPo battery:

- Reading battery voltage.
- Setting charging modes (fast or slow).
- Calculating battery capacity in percentage.
- Registering callbacks for charging state changes.

The library is built on the Zephyr Real-Time Operating System (RTOS). For comprehensive details on Zephyr and how to get started, visit the Zephyr Getting Started Guide.

## Features

### Battery Voltage Reading

- **Function:** `battery_get_millivolt(uint16_t *battery_millivolt)`
- **Description:** Calculates the battery voltage using the ADC and stores the value in the provided pointer.
- **Usage:**

```
uint16_t voltage;
int ret = battery_get_millivolt(&voltage);
if (ret == 0) {
    // Use the voltage value here
}
```

### Charging Modes

- **Set Fast Charge (Default)**
  - **Function:** `battery_set_fast_charge(void)`
  - **Description:** Sets the battery charging current to fast charge mode (100mA).
  - **Usage:**

```
battery_set_fast_charge();
```

- **Set Slow Charge**
  - **Function:** `battery_set_slow_charge(void)`
  - **Description:** Sets the battery charging current to slow charge mode (50mA).
  - **Usage:**

```
battery_set_fast_charge();
```

### Battery Percentage Calculation

- **Function:** `battery_get_percentage(uint8_t *battery_percentage, uint16_t battery_millivolt)`
- **Description:** Calculates the battery percentage based on the voltage and stores it in the provided pointer.
- **Usage:**

```
uint8_t percentage;
uint16_t voltage;
battery_get_millivolt(&voltage);
battery_get_percentage(&percentage, voltage);
```

### Charging State Change Callback

- **Function:** `battery_register_charging_changed_callback(battery_charging_changed_callback_t callback)`
- **Description:** Registers a callback function that is executed whenever the charging state changes.
- **Usage:**

```
void charging_state_changed(bool is_charging) {
    if (is_charging) {
        // Charging started
    } else {
        // Charging stopped
    }
}

battery_register_charging_changed_callback(charging_state_changed);
```

### Initialization

- **Function:** `battery_init(void)`
- **Description:** Initializes the battery charging circuit. Must be called before using other battery functions.
- **Usage:**

```
battery_init();
```

### Example Usage

There is an example in the `main.c` file that demonstrates how to use the battery management library. This example initializes the battery management library, registers a callback to log charging state changes, and periodically reads and logs the battery voltage and capacity.

## Programming

### Adafruit nRF52 Bootloader

The XIAO BLE Sense is equipped with the Adafruit nRF52 Bootloader, which supports UF2 flashing—a simple drag-and-drop method to program your device.

#### Entering Bootloader Mode

1. Use a USB-C cable to connect to the XIAO BLE to your computer
2. **Double-Click** the Reset button (located to the left of the USB connector) quickly. The device should enter bootloader mode and appear as a mass storage device named XIAO on your computer. If the device doesn't appear, ensure your USB cable supports data transfer (some cables are charge-only), and check your computer's device manager or disk utility for new devices.

#### Flashing the Firmware

- Navigate to the '**build/zephyr/**' directory and locate the '**zephyr.uf2**' file. If you can not find the build folder, build or rebuild the project.
- Drag and drop the **zephyr.uf2** file into the XIAO drive that appeared when the device entered bootloader mode or copy the file using your command line.

After the UF2 file transfer is complete, the XIAO BLE will automatically reset and launch the new application.

For additional information on the flashing process and the XIAO BLE Sense board, refer to
the [Zephyr Board Documentation for XIAO BLE](https://docs.zephyrproject.org/latest/boards/seeed/xiao_ble/doc/index.html).

## Serial Logging

Connect to the device's serial port to view log messages from the device. Use a serial terminal application (e.g., PuTTY, Tera Term, Minicom) with the following settings:

- **Baud Rate:** 115200
- **Data Bits:** 8
- **Parity:** None
- **Stop Bits:** 1
