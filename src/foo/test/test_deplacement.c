#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "control/control.h"
#include "kernel/systick.h"
#include "kernel/event.h"
#include "gpio.h"
#include "location/location.h"

//! @todo réglage au pif
#define TEST_DEPLACEMENT_STACK_SIZE       100

static void test_deplacement_task();
int test_deplacement_module_init();

int test_deplacement_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(test_deplacement_task, "t_depl", TEST_DEPLACEMENT_STACK_SIZE, NULL, PRIORITY_TASK_STRATEGY, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_TEST;
	}

	return 0;
}

module_init(test_deplacement_module_init, INIT_TEST_DEPLACEMENT);

static void test_deplacement_task()
{
//	if(getcolor() == COLOR_BLUE)
	{
//		location_set_position(-1410.0f, -850.0f, 0.0f);
	}

	vTaskWaitEvent(EVENT_GO, portMAX_DELAY);

	while(1)
	{
		control_straight(100);
		vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
		control_rotate(1.57);
		vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
		control_straight(-1.57);
		vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
		control_rotate(-100);
		vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	}

	vTaskDelete(NULL);
}
