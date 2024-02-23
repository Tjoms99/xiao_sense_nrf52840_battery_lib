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

#ifndef __BATTERY_H__
#define __BATTERY_H__

typedef enum batt_charging_status{disconnected=-1,notCharging=0,Charging=1} enum_bachast; 
typedef enum batt_max_charge_current{Current0=0, Current50=50,Current100=100} enum_bachacurr;

/**
 * @brief Set battery max charging current {Current0=0, Current50=1,Current100=2}
 *
 * @retval 0 if successful. Negative errno number on error.
 */
int battery_set_max_charge_current(enum_bachacurr max_charge_current);

/**
 * @brief Calculates the battery voltage using the ADC.
 *
 * @param[in] battery_charging_status Pointer to where status has to be stored.
 *
 * @retval 0 if successful. Negative errno number on error.
 */
void battery_get_charging_status(enum_bachast *battery_charging_status);

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
 * @brief Initialize the battery charging circuit.
 *
 * @retval 0 if successful. Negative errno number on error.
 */
int battery_init(void);

#endif