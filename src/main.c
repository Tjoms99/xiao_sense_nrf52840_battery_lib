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
#include <zephyr/drivers/gpio.h>

//LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG|LOG_LEVEL_INF);
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define GPIO_LED_RED      	26
#define GPIO_LED_GREEN      30
#define GPIO_LED_BLUE      	6

struct k_work battery_work;
const struct device *gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
int i=0;

void battery_work_handler(struct k_work *work_item)
{
	uint16_t battery_millivolt = 0;
	uint8_t battery_percentage = 0;
	enum_bachast batt_status;

	battery_get_millivolt(&battery_millivolt);
	battery_get_percentage(&battery_percentage, battery_millivolt);
	battery_get_charging_status(&batt_status);

	LOG_INF("Battery at %d mV (capacity %d%%), charging = %d  %d", battery_millivolt, battery_percentage, batt_status,i);
}

int main(void)
{
	int ret = 0;
	k_msleep(1000); // Gives time for the terminal to connect to catch LOG's
	gpio_pin_configure(gpio0_dev,GPIO_LED_RED,GPIO_OUTPUT|GPIO_ACTIVE_LOW);
	gpio_pin_configure(gpio0_dev,GPIO_LED_GREEN,GPIO_OUTPUT|GPIO_ACTIVE_LOW);
	gpio_pin_configure(gpio0_dev,GPIO_LED_BLUE,GPIO_OUTPUT|GPIO_ACTIVE_LOW);

	ret |= battery_init();

	if (ret)	{
		LOG_ERR("Failed to initialize");
	}
	else	{
		LOG_INF("Initialized");
	}

	k_work_init(&battery_work, battery_work_handler);

	while (1)
	{
		for (i = 0; i<3; i++){
			k_msleep(4000);
			battery_set_max_charge_current(i);
			if (i==0){
				gpio_pin_set(gpio0_dev,GPIO_LED_RED,0);
				gpio_pin_set(gpio0_dev,GPIO_LED_GREEN,0);
				gpio_pin_set(gpio0_dev,GPIO_LED_BLUE,1);
				battery_set_max_charge_current(Current0);
			}
			else if (i==1){
				gpio_pin_set(gpio0_dev,GPIO_LED_RED,0);
				gpio_pin_set(gpio0_dev,GPIO_LED_GREEN,1);
				gpio_pin_set(gpio0_dev,GPIO_LED_BLUE,0);
				battery_set_max_charge_current(Current50);
			}
			else if (i==2){
				gpio_pin_set(gpio0_dev,GPIO_LED_RED,1);
				gpio_pin_set(gpio0_dev,GPIO_LED_GREEN,0);
				gpio_pin_set(gpio0_dev,GPIO_LED_BLUE,0);
				battery_set_max_charge_current(Current100);
			}
			k_msleep(1000);
			k_work_submit(&battery_work);
		}
	}
	return 0;
}