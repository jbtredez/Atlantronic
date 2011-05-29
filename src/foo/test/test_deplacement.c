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
		location_set_position(-880.0f, -880.0f, PI/2.0f);
	}
	else
	{
		location_set_position(880.0f, -880.0f, PI/2.0f);
	}

	// TODO regler la vitesse
	control_straight_to_wall(-200);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	control_straight(120);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	control_rotate(-PI/2.0f);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	control_straight_to_wall(-800);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	control_straight(110);

	while(1) ;

	while(1)
	{
		control_straight(700);
		vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
		vTaskDelay(ms_to_tick(2000));
		/*control_rotate(1.57);
		vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
		control_rotate(-1.57);
		vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);*/
		control_straight(-700);
		vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
		vTaskDelay(ms_to_tick(2000));
	}

	vTaskDelete(NULL);
}
