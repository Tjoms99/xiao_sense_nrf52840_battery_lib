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

#ifndef __BATTERY_H__
#define __BATTERY_H__

#include <stdint.h>
#include <stdbool.h>

// Callback function type definitions
typedef void (*battery_charging_callback_t)(bool is_charging);
typedef void (*battery_sample_callback_t)(uint16_t millivolt);

/**
 * @brief Register a callback function that is executed every time the charging state changes.
 *
 * @retval 0 if successful. Negative errno number on error.
 *
 * @note If the error is -12, try increasing the BATTERY_CALLBACK_MAX define in the library.
 */
int battery_register_charging_callback(battery_charging_callback_t callback);

/**
 * @brief Register a callback function that is executed every time a voltage sample is ready.
 *
 * @retval 0 if successful. Negative errno number on error.
 *
 * @note If the error is -12, try increasing the BATTERY_CALLBACK_MAX define in the library.
 */
int battery_register_sample_callback(battery_sample_callback_t callback);

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
 * @brief Get the current battery voltage in millivolts.
 *
 * @param[out] battery_millivolt Pointer where the battery voltage will be stored.
 *
 * @retval 0 if successful. Negative errno number on error.
 */
int battery_get_millivolt(uint16_t *battery_millivolt);

/**
 * @brief Calculate the battery percentage based on the voltage.
 *
 * @param[out] battery_percentage Pointer where the battery percentage will be stored.
 * @param[in] battery_millivolt Voltage reading to calculate the percentage from.
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
 * @note Registered sample callbacks are called when a new sample is ready.
 */
int battery_start_sampling(uint32_t interval_ms);

/**
 * @brief Stop periodic sampling of the battery voltage.
 *
 * @retval 0 if successful. Negative errno number on error.
 */
int battery_stop_sampling(void);

/**
 * @brief Take a one-time battery voltage sample.
 *
 * @retval 0 if successful. Negative errno number on error.
 *
 * @note Registered sample callbacks are called when the sample is ready.
 */
int battery_sample_once(void);

/**
 * @brief Initialize the battery management system.
 *
 * @retval 0 if successful. Negative errno number on error.
 */
int battery_init(void);

#endif
