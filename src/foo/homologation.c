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
#include "control/control_pince.h"

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

void recalage()
{
	if(getcolor() == COLOR_BLUE)
	{
		location_set_position(-880.0f, -880.0f, PI/2.0f);
	}
	else
	{
		location_set_position(880.0f, -880.0f, PI/2.0f);
	}

	pince_configure();
	pince_close();

	control_pince_independant(1500, 1500);

	// TODO demenager : procedure recalage
	control_straight_to_wall(-200);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	control_straight(200 + PARAM_NP_X);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	control_rotate(-PI/2.0f);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	control_straight_to_wall(-800);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	control_straight(110);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
}

static void test_deplacement_task()
{
	while(getGo() == 0)
	{
		if( getRecalage() )
		{
			recalage();
			resetRecalage();
		}
		vTaskDelay(ms_to_tick(50));
	}

	vTaskWaitEvent(EVENT_GO, portMAX_DELAY);

//	vTaskDelay(ms_to_tick(5000));
	vTaskWaitEvent(EVENT_HOKUYO_READY, ms_to_tick(1000));

	control_goto(-875, -850);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	pince_open();
	control_pince_dual(70, 0);
//	control_goto(-700, -350);
	control_goto_near(-700, -350, 160);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);

	pince_close();
	vTaskDelay(ms_to_tick(400));

	control_pince_dual(3000, 0);
	vTaskDelay(ms_to_tick(1000));

	control_goto_near(-350, -700, 160);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	pince_open();
	vTaskDelay(ms_to_tick(300));
	control_pince_dual(70, 0);
	vTaskDelay(ms_to_tick(1000));
//	vTaskWaitEvent(EVENT_CONTROL_PINCE_READY, portMAX_DELAY);
	pince_close();
	vTaskDelay(ms_to_tick(400));
	control_pince_dual(3000, 0);

	while(1) 
	{
		pince_close();
		vTaskDelay(ms_to_tick(1000));
	}

	vTaskDelete(NULL);
}

