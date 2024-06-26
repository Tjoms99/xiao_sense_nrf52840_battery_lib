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

#include <stdint.h>
#include <stdbool.h>

#ifndef __BATTERY_H__
#define __BATTERY_H__

// Callback function type definition
typedef void (*battery_charging_changed_callback_t)(bool is_charging);
typedef void (*battery_sample_ready_callback_t)(uint16_t millivolt);

/**
 * @brief Register a callback function which is executed every time the charging state is changed.
 *
 * @retval 0 if successful. Negative errno number on error.
 *
 * @note If the error is -12, try to increase the BATTERY_CALLBACK_MAX define in the libray.
 */
int battery_register_charging_changed_callback(battery_charging_changed_callback_t callback);

/**
 * @brief Register a callback function which is executed every time a voltage sample is ready.
 *
 * @retval 0 if successful. Negative errno number on error.
 *
 * @note If the error is -12, try to increase the BATTERY_CALLBACK_MAX define in the libray.
 */
int battery_register_sample_ready_callback(battery_sample_ready_callback_t callback);

/**
 * @brief Set battery charging to fast charge (100mA).
 *
 * @retval 0 if successful. Negative errno number on error.
 */
int battery_set_fast_charge(void);

/**
 * @brief Set battery charging to slow charge (50mA).
 *
 * @retval 0 if successful. Negative errno number on error.
 */
int battery_set_slow_charge(void);

/**
 * @brief Calculates the battery voltage using the ADC.
 *
 * @param[in] battery_millivolt Pointer to where battery voltage is stored.
 *
 * @retval 0 if successful. Negative errno number on error.
 */
int battery_get_millivolt(uint16_t *battery_millivolt);

/**
 * @brief Calculates the battery percentage using the battery voltage.
 *
 * @param[in] battery_percentage  Pointer to where battery percentage is stored.
 *
 * @param[in] battery_millivolt Voltage used to calculate the percentage of how much energy is left in a 3.7V LiPo battery.
 *
 * @retval 0 if successful. Negative errno number on error.
 */
int battery_get_percentage(uint8_t *battery_percentage, uint16_t battery_millivolt);

/**
 * @brief Start periodic sampling of the battery voltage.
 *
 * @param[in] interval_ms Sampling interval in milliseconds.
 *
 * @retval 0 if successful. Negative errno number on error.
 *
 * @note The callbacks registered in "battery_register_sample_ready_callback" are run when the voltage reading is done.
 */
int battery_start_periodic_sampling(uint32_t interval_ms);

/**
 * @brief Stop periodic sampling of the battery voltage.
 *
 * @retval 0 if successful. Negative errno number on error.
 */
int battery_stop_periodic_sampling(void);

/**
 * @brief Sample one voltage reading.
 *
 * @retval 0 if successful. Negative errno number on error.
 *
 * @note The callbacks registered in "battery_register_sample_ready_callback" are run when the voltage reading is done.
 */
int battery_start_one_shot_sample(void);

/**
 * @brief Initialize the battery charging circuit.
 *
 * @retval 0 if successful. Negative errno number on error.
 */
int battery_init(void);

#endif