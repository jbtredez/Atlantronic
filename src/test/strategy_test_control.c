#include "FreeRTOS.h"
#include "task.h"
#include "module.h"
#include "control.h"
#include "time2.h"
#include "event.h"

//! @todo réglage au pif
#define STRATEGY_TEST_CONTROL_STACK_SIZE       10

static void strategy_test_control_task();
int strategy_test_control_module_init();

int strategy_test_control_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(strategy_test_control_task, (const signed char *) "strategy_test_control", STRATEGY_TEST_CONTROL_STACK_SIZE, NULL, PRIORITY_TASK_STRATEGY, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_CONTROL;
	}

	return 0;
}

module_init(strategy_test_control_module_init, INIT_STRATEGY);

static void strategy_test_control_task()
{
	time_start_match();

	vTaskClearEvent(EVENT_CONTROL_READY);
	control_goto(1000,-1000);
	vTaskWaitEvent(EVENT_CONTROL_READY);
	control_goto(1500,-1000);

	vTaskDelete(NULL);
}