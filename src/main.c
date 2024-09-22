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

#include "battery/battery.h"

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

void log_battery_voltage(uint16_t millivolt)
{
	uint8_t battery_percentage = 0;
	int ret = battery_get_percentage(&battery_percentage, millivolt);
	if (ret == 0)
	{
		LOG_INF("Battery at %d mV (capacity %d%%)", millivolt, battery_percentage);
	}
	else
	{
		LOG_ERR("Failed to calculate battery percentage");
	}
}

void log_charging_state(bool is_charging)
{
	LOG_INF("Charger %s", is_charging ? "connected" : "disconnected");
}

int main(void)
{
	int ret = 0;
	k_msleep(1000); // Gives time for the terminal to connect to catch logs

	ret = battery_init();
	if (ret)
	{
		LOG_ERR("Failed to initialize battery management (error %d)", ret);
		return ret;
	}

	ret = battery_register_charging_callback(log_charging_state);
	if (ret)
	{
		LOG_ERR("Failed to register charging callback (error %d)", ret);
		return ret;
	}

	ret = battery_register_sample_callback(log_battery_voltage);
	if (ret)
	{
		LOG_ERR("Failed to register sample callback (error %d)", ret);
		return ret;
	}

	// Take a one-time sample
	battery_sample_once();

	// Start periodic sampling every 1000 ms
	battery_start_sampling(1000);

	while (1)
	{
		k_sleep(K_SECONDS(10));
		// You can stop periodic sampling if needed
		// battery_stop_sampling();
	}

	return 0;
}