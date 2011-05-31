#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "kernel/systick.h"
#include "kernel/event.h"
#include "kernel/rcc.h"
#include "bar/gpio.h"

//! @todo réglage au pif
#define US_STACK_SIZE       100

static void us_task();
int us_module_init();

int us_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(us_task, "us", US_STACK_SIZE, NULL, PRIORITY_TASK_TEST_US, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_CONTROL;
	}

	return 0;
}

module_init(us_module_init, INIT_TEST_PINCE);

static void us_task()
{
	while(1)
	{
		gpio_send_us(GPIO_US0 | GPIO_US1 | GPIO_US2 | GPIO_US3 | GPIO_US4);

		vTaskDelay(ms_to_tick(500));
	}

	vTaskDelete(NULL);
}

