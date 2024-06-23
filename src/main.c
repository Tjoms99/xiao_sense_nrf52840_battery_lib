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

struct k_work battery_work;

void battery_work_handler(struct k_work *work_item)
{
	uint16_t battery_millivolt = 0;
	uint8_t battery_percentage = 0;

	battery_get_millivolt(&battery_millivolt);
	battery_get_percentage(&battery_percentage, battery_millivolt);

	LOG_INF("Battery at %d mV (capacity %d%%)", battery_millivolt, battery_percentage);
}

void log_charging_state(bool is_charging)
{
	LOG_INF("Charger %s", is_charging ? "connected" : "disconnected");
}

int main(void)
{
	int ret = 0;
	k_msleep(1000); // Gives time for the terminal to connect to catch LOG's

	ret |= battery_init();
	ret |= battery_register_charging_changed_callback(log_charging_state);

	if (ret)
	{
		LOG_ERR("Failed to initialize");
	}
	else
	{
		LOG_INF("Initialized");
	}

	k_work_init(&battery_work, battery_work_handler);

	while (1)
	{
		k_msleep(1000);
		k_work_submit(&battery_work);
	}

	return 0;
}