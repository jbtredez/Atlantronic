#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "control/control.h"
#include "kernel/systick.h"
#include "kernel/event.h"
#include "kernel/driver/hokuyo.h"
#include "gpio.h"
#include "location/location.h"

//! @todo réglage au pif
#define TEST_HOKUYO_STACK_SIZE       100

static void test_hokuyo_task();
int test_hokuyo_module_init();

int test_hokuyo_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(test_hokuyo_task, "t_hoku", TEST_HOKUYO_STACK_SIZE, NULL, PRIORITY_TASK_STRATEGY, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_TEST;
	}

	return 0;
}

module_init(test_hokuyo_module_init, INIT_TEST_DEPLACEMENT);

static void test_hokuyo_task()
{
//	vTaskWaitEvent(EVENT_GO, portMAX_DELAY);

	hokuyo_init();

	while(1)
	{

	}

	vTaskDelete(NULL);
}
