/*
 * Copyright 2024 Marcus Alexander Tjomsaas
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "battery.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(battery, LOG_LEVEL_INF);

static K_THREAD_STACK_DEFINE(battery_workq_stack_area,
                             1024);

#define GPIO_BATTERY_CHARGE_SPEED 13
#define GPIO_BATTERY_CHARGING_ENABLE 17
#define GPIO_BATTERY_READ_ENABLE 14

// Feel free to increase this when necessary
#define BATTERY_CALLBACK_MAX 3

// Change this to a higher number for better averages
// Note that increasing this holds up the thread / ADC for longer.
#define ADC_TOTAL_SAMPLES 10

//--------------------------------------------------------------
// ADC setup

#define ADC_RESOLUTION 12
#define ADC_CHANNEL 7
#define ADC_PORT SAADC_CH_PSELP_PSELP_AnalogInput7 // AIN7
#define ADC_REFERENCE ADC_REF_INTERNAL             // 0.6V
#define ADC_GAIN ADC_GAIN_1_6                      // ADC REFERENCE * 6 = 3.6V
#define ADC_SAMPLE_INTERVAL_US 500                 // Time between each sample

static struct adc_channel_cfg channel_7_cfg = {
    .gain = ADC_GAIN,
    .reference = ADC_REFERENCE,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id = ADC_CHANNEL,
#ifdef CONFIG_ADC_NRFX_SAADC
    .input_positive = ADC_PORT
#endif
};

static struct adc_sequence_options options = {
    .extra_samplings = ADC_TOTAL_SAMPLES - 1,
    .interval_us = ADC_SAMPLE_INTERVAL_US,
};

static int16_t sample_buffer[ADC_TOTAL_SAMPLES];
static struct adc_sequence sequence = {
    .options = &options,
    .channels = BIT(ADC_CHANNEL),
    .buffer = sample_buffer,
    .buffer_size = sizeof(sample_buffer),
    .resolution = ADC_RESOLUTION,
};

//--------------------------------------------------------------
// Local variables

// MCU peripherals for reading battery voltage
static const struct device *gpio_battery_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
static const struct device *adc_battery_dev = DEVICE_DT_GET(DT_NODELABEL(adc));

// Battery work and work queue
static struct k_work_q battery_workq;
static struct k_work_delayable sample_periodic_work;
static struct k_work sample_once_work;

// Charging interrupt
static struct gpio_callback charging_callback;
static struct k_work charging_interrupt_work;

// Callbacks for change in charging
static battery_charging_callback_t charging_callbacks[BATTERY_CALLBACK_MAX];
static size_t charging_callbacks_registered = 0;

// Callbacks for when a battery sample is ready
static battery_sample_callback_t sample_ready_callback[BATTERY_CALLBACK_MAX];
static size_t sample_ready_callbacks_registered = 0;

static uint32_t sampling_interval_ms;
static uint8_t is_initialized = false;

static K_MUTEX_DEFINE(battery_mut);

typedef struct
{
    uint16_t voltage;
    uint8_t percentage;
} BatteryState;

#define BATTERY_STATES_COUNT 12
// Assuming LiPo battery.
// For better accuracy, use your battery's datasheet.
static BatteryState battery_states[BATTERY_STATES_COUNT] = {
    {4200, 100},
    {4160, 99},
    {4090, 91},
    {4030, 78},
    {3890, 63},
    {3830, 53},
    {3680, 36},
    {3660, 35},
    {3480, 14},
    {3420, 11},
    {3150, 1}, // 3240
    {0000, 0}  // Below safe level
};

//------------------------------------------------------------------------------------------
// Private functions

static int battery_enable_read()
{
    return gpio_pin_set(gpio_battery_dev, GPIO_BATTERY_READ_ENABLE, 1);
}

static void run_charging_callbacks(struct k_work *work)
{
    bool is_charging = gpio_pin_get(gpio_battery_dev, GPIO_BATTERY_CHARGING_ENABLE);
    LOG_DBG("Charger %s", is_charging ? "connected" : "disconnected");

    for (uint8_t callback = 0; callback < charging_callbacks_registered; callback++)
    {
        charging_callbacks[callback](is_charging);
    }
}

static void run_sample_ready_callbacks(uint32_t millivolt)
{

    for (uint8_t callback = 0; callback < sample_ready_callbacks_registered; callback++)
    {
        sample_ready_callback[callback](millivolt);
    }
}

static void charging_callback_handler(const struct device *dev,
                                      struct gpio_callback *cb,
                                      uint32_t pins)
{
    k_work_submit_to_queue(&battery_workq, &charging_interrupt_work);
}

static void sample_periodic_handler(struct k_work *work)
{
    uint16_t millivolt;
    int ret = battery_get_millivolt(&millivolt);
    if (ret)
    {
        LOG_ERR("Failed to get battery voltage");
        goto reschedule;
    }

    // Run all the callbacks waiting for a voltage reading.
    run_sample_ready_callbacks(millivolt);

reschedule:
    k_work_reschedule_for_queue(&battery_workq, &sample_periodic_work, K_MSEC(sampling_interval_ms));
}

static void sample_once_handler(struct k_work *work)
{
    uint16_t millivolt;
    int ret = battery_get_millivolt(&millivolt);
    if (ret)
    {
        LOG_ERR("Failed to get battery voltage");
        return;
    }

    // Run all the callbacks waiting for voltage readings.
    run_sample_ready_callbacks(millivolt);
}

//------------------------------------------------------------------------------------------
// Public functions

int battery_register_charging_callback(battery_charging_callback_t callback)
{
    if (charging_callbacks_registered == BATTERY_CALLBACK_MAX)
    {
        LOG_ERR("Maximum number of callbacks reached, operation aborted");
        return -ENOMEM;
    }

    charging_callbacks[charging_callbacks_registered++] = callback;

    return 0;
}

int battery_register_sample_callback(battery_sample_callback_t callback)
{
    if (sample_ready_callbacks_registered == BATTERY_CALLBACK_MAX)
    {
        LOG_ERR("Maximum number of callbacks reached, operation aborted");
        return -ENOMEM;
    }

    sample_ready_callback[sample_ready_callbacks_registered++] = callback;
    return 0;
}

int battery_set_fast_charge()
{
    if (!is_initialized)
    {
        return -ECANCELED;
    }

    return gpio_pin_set(gpio_battery_dev, GPIO_BATTERY_CHARGE_SPEED, 1); // FAST charge 100mA
}

int battery_set_slow_charge()
{
    if (!is_initialized)
    {
        return -ECANCELED;
    }

    return gpio_pin_set(gpio_battery_dev, GPIO_BATTERY_CHARGE_SPEED, 0); // SLOW charge 50mA
}

int battery_get_millivolt(uint16_t *battery_millivolt)
{

    int ret = 0;

    // Voltage divider circuit (Should tune R1 in software if possible)
    const uint16_t R1 = 1037; // Originally 1M ohm, calibrated after measuring actual voltage values. Can happen due to resistor tolerances, temperature ect..
    const uint16_t R2 = 510;  // 510K ohm

    // ADC measure
    uint16_t adc_vref = adc_ref_internal(adc_battery_dev);
    int adc_mv = 0;

    k_mutex_lock(&battery_mut, K_FOREVER);
    ret |= adc_read(adc_battery_dev, &sequence);

    if (ret)
    {
        LOG_WRN("ADC read failed (error %d)", ret);
    }

    // Get average sample value.
    for (uint8_t sample = 0; sample < ADC_TOTAL_SAMPLES; sample++)
    {
        adc_mv += sample_buffer[sample]; // ADC value, not millivolt yet.
    }
    adc_mv /= ADC_TOTAL_SAMPLES;

    // Convert ADC value to millivolts
    ret |= adc_raw_to_millivolts(adc_vref, ADC_GAIN, ADC_RESOLUTION, &adc_mv);

    // Calculate battery voltage.
    float scale_factor = ((float)(R1 + R2)) / R2;
    *battery_millivolt = (uint16_t)(adc_mv * scale_factor);

    k_mutex_unlock(&battery_mut);

    LOG_DBG("%d mV", *battery_millivolt);
    return ret;
}

int battery_get_percentage(uint8_t *battery_percentage, uint16_t battery_millivolt)
{
    // Ensure voltage is within bounds
    if (battery_millivolt >= battery_states[0].voltage)
    {
        *battery_percentage = 100;
        return 0;
    }
    else if (battery_millivolt <= battery_states[BATTERY_STATES_COUNT - 1].voltage)
    {
        *battery_percentage = 0;
        return 0;
    }

    for (uint16_t i = 0; i < BATTERY_STATES_COUNT - 1; i++)
    {
        uint16_t voltage_high = battery_states[i].voltage;
        uint16_t voltage_low = battery_states[i + 1].voltage;

        // Find the two points between which battery_millivolt lies
        if (battery_millivolt <= voltage_high && battery_millivolt >= voltage_low)
        {
            uint8_t percentage_high = battery_states[i].percentage;
            uint8_t percentage_low = battery_states[i + 1].percentage;

            int32_t voltage_range = voltage_high - voltage_low;          // Should be positive
            int32_t percentage_range = percentage_high - percentage_low; // Should be positive
            int32_t voltage_diff = battery_millivolt - voltage_low;      // Non-negative

            if (voltage_range == 0)
            {
                *battery_percentage = percentage_high;
            }
            else
            {
                *battery_percentage = percentage_low + (voltage_diff * percentage_range) / voltage_range;
            }

            LOG_DBG("%d %%", *battery_percentage);
            return 0;
        }
    }

    // If voltage is not within any defined range
    return -ESPIPE;
}

int battery_start_sampling(uint32_t interval_ms)
{
    sampling_interval_ms = interval_ms;
    k_work_schedule_for_queue(&battery_workq, &sample_periodic_work, K_MSEC(interval_ms));

    LOG_INF("Start sampling battery voltage at %d ms", interval_ms);
    return 0;
}

int battery_stop_sampling(void)
{
    k_work_cancel_delayable(&sample_periodic_work);
    LOG_INF("Stopped periodic sampling of battery voltage");
    return 0;
}

int battery_sample_once(void)
{
    k_work_submit_to_queue(&battery_workq, &sample_once_work);
    return 0;
}

int battery_init()
{
    int ret = 0;

    // ADC setup
    if (!device_is_ready(adc_battery_dev))
    {
        LOG_ERR("ADC device not found!");
        return -EIO;
    }

    ret |= adc_channel_setup(adc_battery_dev, &channel_7_cfg);
    if (ret)
    {
        LOG_ERR("ADC setup failed (error %d)", ret);
        return ret;
    }

    // GPIO setup
    if (!device_is_ready(gpio_battery_dev))
    {
        LOG_ERR("GPIO device not found!");
        return -EIO;
    }

    ret |= gpio_pin_configure(gpio_battery_dev, GPIO_BATTERY_CHARGING_ENABLE, GPIO_INPUT | GPIO_ACTIVE_LOW);
    if (ret)
    {
        LOG_ERR("Failed to configure GPIO_BATTERY_CHARGING_ENABLE pin (error %d)", ret);
        return ret;
    }

    ret |= gpio_pin_interrupt_configure(gpio_battery_dev, GPIO_BATTERY_CHARGING_ENABLE, GPIO_INT_EDGE_BOTH);
    if (ret)
    {
        LOG_ERR("Failed to configure GPIO_BATTERY_CHARGING_ENABLE pin interrupt (error %d)", ret);
        return ret;
    }

    ret |= gpio_pin_configure(gpio_battery_dev, GPIO_BATTERY_READ_ENABLE, GPIO_OUTPUT | GPIO_ACTIVE_LOW);
    if (ret)
    {
        LOG_ERR("Failed to configure GPIO_BATTERY_READ_ENABLE pin (error %d)", ret);
        return ret;
    }
    ret |= gpio_pin_configure(gpio_battery_dev, GPIO_BATTERY_CHARGE_SPEED, GPIO_OUTPUT | GPIO_ACTIVE_LOW);
    if (ret)
    {
        LOG_ERR("Failed to configure GPIO_BATTERY_CHARGE_SPEED pin (error %d)", ret);
        return ret;
    }

    // Battery work and work queue setup
    k_work_queue_start(&battery_workq, battery_workq_stack_area,
                       K_THREAD_STACK_SIZEOF(battery_workq_stack_area), CONFIG_SYSTEM_WORKQUEUE_PRIORITY, NULL);
    k_work_init_delayable(&sample_periodic_work, sample_periodic_handler);
    k_work_init(&sample_once_work, sample_once_handler);

    // Charger interrupt setup
    k_work_init(&charging_interrupt_work, run_charging_callbacks);
    gpio_init_callback(&charging_callback, charging_callback_handler,
                       BIT(GPIO_BATTERY_CHARGING_ENABLE));
    gpio_add_callback(gpio_battery_dev, &charging_callback);

    // Lets check the current charging status
    bool is_charging = gpio_pin_get(gpio_battery_dev, GPIO_BATTERY_CHARGING_ENABLE);
    LOG_INF("Charger %s", is_charging ? "connected" : "disconnected");

    is_initialized = true;
    LOG_INF("Initialized");

    // Get ready for battery charging and sampling
    ret |= battery_set_fast_charge();
    if (ret)
    {
        LOG_ERR("Failed to set fast charging (error %d)", ret);
        return ret;
    }

    ret |= battery_enable_read();
    if (ret)
    {
        LOG_ERR("Failed to enable battery reading (error %d)", ret);
        return ret;
    }

    return 0;
}
