
#include "battery/battery.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

struct k_work battery_work;

void battery_work_handler(struct k_work *work_tem)
{
	float battery_volt = 0;
	int battery_percentage = 0;

	battery_get_voltage(&battery_volt);
	battery_get_percentage(&battery_percentage, battery_volt);

	LOG_INF("Battery capacity %d%%", battery_percentage);
}

int main(void)
{
	int ret = 0;
	k_msleep(1000); // Gives time for the terminal to connect to catch LOG's

	ret |= battery_init();
	ret |= battery_charge_start();

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