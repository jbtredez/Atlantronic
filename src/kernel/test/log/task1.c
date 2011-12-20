//! @file task1.c
//! @brief Tests : test des log
//! @author Atlantronic

#include "kernel/module.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/event.h"
#include "kernel/rcc.h"
#include "kernel/log.h"

#define TASK1_STACK_SIZE          250
#define PRIORITY_TEST_TASK1         1

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

	(void) arg;
	int i = 0;

	portTickType wake = 0;//systick_get_time();

	while(1)
	{
		log(LOG_INFO, "test log : info");
		log(LOG_ERROR, "test log : error");
		log(LOG_DEBUG1, "test log : debug_1");
		log(LOG_DEBUG2, "test log : debug_2");
		log_format(LOG_INFO, "test log : info %i", i);
		log_format(LOG_ERROR, "test log : error %i", i);
		log_format(LOG_DEBUG1, "test log : debug_1 %i", i);
		log_format(LOG_DEBUG2, "test log : debug_2 %i", i);
		i++;
		wake += ms_to_tick(1000);
		vTaskDelayUntil(wake);
	}

	vTaskDelete(NULL);
}
