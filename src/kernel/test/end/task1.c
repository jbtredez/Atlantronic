//! @file task1.c
//! @brief Tests : test des led
//! @author Atlantronic

#include "kernel/module.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include <math.h>
#include "gpio.h"
#include "kernel/end.h"
#include "kernel/event.h"

#define TASK1_STACK_SIZE          100
#define PRIORITY_TEST_TASK1        1

static void task1_task(void *arg);

static int task1_module_init()
{
	xTaskHandle xHandle;

	portBASE_TYPE err = xTaskCreate(task1_task, "task1", TASK1_STACK_SIZE, NULL, PRIORITY_TEST_TASK1, &xHandle);

	if(err != pdPASS)
	{
		return -1;
	}

	return 0;
}

module_init(task1_module_init, INIT_TEST_TASK1);

static void task1_task(void* arg)
{
	(void) arg;

	gpio_wait_go();

	while(1)
	{
		if(! end_match)
		{
			setLed(LED_WARNING);
		}

		vTaskDelay(72000000);
		
		if(! end_match)
		{
			setLed(LED_0);
		}
		
		vTaskDelay(72000000);
		
		if(! end_match)
		{
			setLed(LED_1);
		}
		
		vTaskDelay(72000000);
		
		if(! end_match)
		{
			setLed(LED_2);
		}
		
		vTaskDelay(72000000);
		
		if(! end_match)
		{
			setLed(LED_3);
		}
		
		vTaskDelay(72000000);
		
		if(! end_match)
		{
			setLed(LED_4);
		}
		
		vTaskDelay(72000000);
		
		if(! end_match)
		{
			setLed(LED_5);
		}
		
		vTaskDelay(72000000);
	}

	vTaskDelete(NULL);
}
