#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "control/control.h"
#include "kernel/systick.h"
#include "kernel/event.h"
#include "gpio.h"
#include "location/location.h"
#include "kernel/rcc.h"
#include <math.h>
#include "kernel/robot_parameters.h"
#include "pince.h"

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
	vTaskWaitEvent(EVENT_GO, portMAX_DELAY);

	if(getcolor() == COLOR_BLUE)
	{
		location_set_position(-700.0f, -700.0f, 0);
	}
	else
	{
		location_set_position(700.0f, -700.0f, PI);
	}

	//pince_configure();
	//pince_close();
	//control_pince_dual(1000, 0);
#if 0
	while(1)
	{
		control_goto_near(-700.0f, -700.0f, 0, TRAJECTORY_ANY_WAY);
		vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
		control_goto_near(-700.0f, 350.0f, 0, TRAJECTORY_ANY_WAY);
		vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
		control_goto_near(700.0f, 350.0f, 0, TRAJECTORY_ANY_WAY);
		vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
		control_goto_near(700.0f, -700.0f, 0, TRAJECTORY_ANY_WAY);
		vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	}
#endif
	vTaskDelete(NULL);
}
