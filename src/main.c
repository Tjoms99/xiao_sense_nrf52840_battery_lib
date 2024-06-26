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

void log_battery_millivolt(uint16_t millivolt)
{
	uint8_t battery_percentage = 0;
	battery_get_percentage(&battery_percentage, millivolt);

	LOG_INF("Battery at %d mV (capacity %d%%)", millivolt, battery_percentage);
}

void log_battery_charging_state(bool is_charging)
{
	LOG_INF("Charger %s", is_charging ? "connected" : "disconnected");
}

int main(void)
{
	int ret = 0;
	k_msleep(1000); // Gives time for the terminal to connect to catch LOG's

	ret |= battery_init();
	ret |= battery_register_charging_changed_callback(log_battery_charging_state);
	ret |= battery_register_sample_ready_callback(log_battery_millivolt);

	if (ret)
	{
		LOG_ERR("Failed to initialize (error %d)", ret);
	}
	else
	{
		LOG_INF("Initialized");
	}

	// This is getting one sample only
	battery_start_one_shot_sample();

	while (1)
	{
		battery_start_periodic_sampling(1000);
		k_sleep(K_SECONDS(10));
		battery_stop_periodic_sampling();
		k_sleep(K_SECONDS(5));
	}

	return 0;
}