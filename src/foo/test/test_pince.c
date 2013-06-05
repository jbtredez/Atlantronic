#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "control/control.h"
#include "kernel/systick.h"
#include "kernel/event.h"
#include "kernel/rcc.h"
#include "gpio.h"
#include "pince.h"

//! @todo réglage au pif
#define TEST_PINCE_STACK_SIZE       100

static void test_pince_task();
int test_pince_module_init();

int test_pince_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(test_pince_task, "t_pince", TEST_PINCE_STACK_SIZE, NULL, PRIORITY_TASK_STRATEGY, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_CONTROL;
	}

	return 0;
}

module_init(test_pince_module_init, INIT_TEST_PINCE);

static void test_pince_task()
{
	gpio_wait_go();

	pince_set_position(PINCE_OPEN,PINCE_OPEN);
	vTaskDelay(ms_to_tick(1000));

	while(1)
	{
		pince_set_position(PINCE_CLOSE,PINCE_CLOSE);
		vTaskDelay(ms_to_tick(2000));
		pince_set_position(PINCE_OPEN,PINCE_OPEN);
		vTaskDelay(ms_to_tick(2000));
	}

	vTaskDelete(NULL);
}

