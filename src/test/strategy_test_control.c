#include "FreeRTOS.h"
#include "task.h"
#include "module.h"
#include "control.h"

//! @todo réglage au pif
#define STRATEGY_TEST_CONTROL_STACK_SIZE       10

static void strategy_test_control_task();
static int strategy_test_control_module_init();

static int strategy_test_control_module_init()
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
	control_straight(1000);

	vTaskDelete(NULL);
}
