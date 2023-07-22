#include "battery.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(battery, LOG_LEVEL_DBG);

static const struct device *gpio_battery_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
static const struct device *adc_battery_dev = DEVICE_DT_GET(DT_NODELABEL(adc));

#define GPIO_BATTERY_CHARGE_SPEED 13
#define GPIO_BATTERY_CHARGING_ENABLE 17
#define GPIO_BATTERY_READ_ENABLE 14

#define ADC_TOTAL_SAMPLES 10
int16_t sample_buffer[ADC_TOTAL_SAMPLES];

#define ADC_RESOLUTION 12
#define ADC_CHANNEL 7
#define ADC_PORT SAADC_CH_PSELP_PSELP_AnalogInput7 // AIN7
#define ADC_REFERENCE ADC_REF_INTERNAL             // 0.6V
#define ADC_GAIN ADC_GAIN_1_6                      // ADC REFERENCE * 6 = 3.6V

struct adc_channel_cfg channel_7_cfg = {
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
    .interval_us = 500, // Interval between each sample
};

struct adc_sequence sequence = {
    .options = &options,
    .channels = BIT(ADC_CHANNEL),
    .buffer = sample_buffer,
    .buffer_size = sizeof(sample_buffer),
    .resolution = ADC_RESOLUTION};

typedef struct
{
    float voltage;
    int percentage;
} BatteryState;

#define BATTERY_STATES_COUNT 12
// Assuming LiPo battery.
// Source: https://forum.evolvapor.com/topic/65565-discharge-profiles-csv-for-2-3-18650-batteries-sony-vtc6-sammy-30q/
BatteryState battery_states[BATTERY_STATES_COUNT] = {
    {4.20, 100},
    {4.16, 99},
    {4.09, 91},
    {4.03, 78},
    {3.89, 63},
    {3.83, 53},
    {3.68, 36},
    {3.66, 35},
    {3.48, 14},
    {3.42, 11},
    {3.15, 1}, // .24
    {0.00, 0}  // Below safe level
};

static uint8_t is_initialized = false;

static int battery_enable_read()
{
    return gpio_pin_set(gpio_battery_dev, GPIO_BATTERY_READ_ENABLE, 1);
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

int battery_charge_start()
{
    int ret = 0;

    if (!is_initialized)
    {
        return -ECANCELED;
    }
    ret |= battery_enable_read();
    ret |= gpio_pin_set(gpio_battery_dev, GPIO_BATTERY_CHARGING_ENABLE, 1);
    return ret;
}

int battery_charge_stop()
{
    if (!is_initialized)
    {
        return -ECANCELED;
    }

    return gpio_pin_set(gpio_battery_dev, GPIO_BATTERY_CHARGING_ENABLE, 0);
}

int battery_get_voltage(float *battery_volt)
{

    int ret = 0;

    // Voltage divider circuit
    const int R1 = 1037; // Originally 1M ohm, calibrated after measuring actual voltage values. Can happen due to resistor tolerances, temperature ect..
    const int R2 = 510;  // 510K ohm

    // ADC measure
    int32_t adc_vref = adc_ref_internal(adc_battery_dev);
    int battery_millivolt = 0;
    int adc_mv = 0;

    ret |= adc_read(adc_battery_dev, &sequence);
    if (ret)
    {
        LOG_WRN("ADC read failed (error %d)", ret);
    }

    // Get average sample value.
    for (int sample = 0; sample < ADC_TOTAL_SAMPLES; sample++)
    {
        adc_mv += sample_buffer[sample]; // ADC value, not millivolt yet.
    }
    adc_mv /= ADC_TOTAL_SAMPLES;

    // Convert sample value to millivolts
    ret |= adc_raw_to_millivolts(adc_vref, ADC_GAIN, ADC_RESOLUTION, &adc_mv);

    // Calculate battery voltage.
    battery_millivolt = adc_mv * ((R1 + R2) / R2);
    *battery_volt = (float)battery_millivolt / 1000.0; // From millivolt to volt.

    LOG_DBG("%d mV", battery_millivolt);
    return ret;
}

int battery_get_percentage(int *battery_percentage, float battery_voltage)
{

    // Ensure voltage is within bounds
    if (battery_voltage > battery_states[0].voltage)
        *battery_percentage = 100;
    if (battery_voltage < battery_states[BATTERY_STATES_COUNT - 1].voltage)
        *battery_percentage = 0;

    for (int i = 0; i < BATTERY_STATES_COUNT - 1; i++)
    {
        // Find the two points battery_voltage is between
        if (battery_states[i].voltage >= battery_voltage && battery_voltage >= battery_states[i + 1].voltage)
        {
            // Linear interpolation
            *battery_percentage = battery_states[i].percentage +
                                  ((battery_voltage - battery_states[i].voltage) *
                                   ((battery_states[i + 1].percentage - battery_states[i].percentage) /
                                    (battery_states[i + 1].voltage - battery_states[i].voltage)));

            LOG_DBG("%d %%", *battery_percentage);
            return 0;
        }
    }
    return -ESPIPE;
}

int battery_init()
{
    int ret = 0;

    // ADC
    if (!device_is_ready(adc_battery_dev))
    {
        LOG_ERR("ADC device not found!");
        return -EIO;
    }

    ret |= adc_channel_setup(adc_battery_dev, &channel_7_cfg);

    if (ret)
    {
        LOG_ERR("ADC setup failed (error %d)", ret);
    }

    // GPIO
    if (!device_is_ready(gpio_battery_dev))
    {
        LOG_ERR("GPIO device not found!");
        return -EIO;
    }

    ret |= gpio_pin_configure(gpio_battery_dev, GPIO_BATTERY_CHARGING_ENABLE, GPIO_OUTPUT | GPIO_ACTIVE_LOW);
    ret |= gpio_pin_configure(gpio_battery_dev, GPIO_BATTERY_READ_ENABLE, GPIO_OUTPUT | GPIO_ACTIVE_LOW);
    ret |= gpio_pin_configure(gpio_battery_dev, GPIO_BATTERY_CHARGE_SPEED, GPIO_OUTPUT | GPIO_ACTIVE_LOW);

    if (ret)
    {
        LOG_ERR("GPIO configure failed123!");
        return ret;
    }

    if (ret)
    {
        LOG_ERR("Initialization failed (error %d)", ret);
        return ret;
    }

    is_initialized = true;
    LOG_INF("Initialized");

    ret |= battery_enable_read();
    ret |= battery_set_fast_charge();

    return ret;
}