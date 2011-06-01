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
#include "recalage.h"
#include "kernel/us.h"

#define HOMOLOGATION_STACK_SIZE       100

static void homologation_task();
int homologation_module_init();

int homologation_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(homologation_task, "homol", HOMOLOGATION_STACK_SIZE, NULL, PRIORITY_TASK_STRATEGY, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_TEST;
	}

	return 0;
}

module_init(homologation_module_init, INIT_STRATEGY);

static void homologation_task()
{
	uint32_t event;
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

	pince_open();
	control_pince_dual(70, 0);

	//us_set_activated(US_FRONT_MASK);

	do
	{
//		control_free();
		control_goto(0, -700);
		event = vTaskWaitEvent(EVENT_CONTROL_READY | EVENT_US_COLLISION, portMAX_DELAY);
	}while(event & EVENT_US_COLLISION);

	do
	{
//		control_free();
		control_goto_near(0, 0, 160);
		event = vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	}while(event & EVENT_US_COLLISION);

	pince_close();
	vTaskDelay(ms_to_tick(300));
	control_pince_dual(1500, 0);
	vTaskDelay(ms_to_tick(300));

	do
	{
//		control_free();
//		control_goto_near(175, -175, 160);
		control_goto_near(525, 175, 160);
		event = vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	}while(event & EVENT_US_COLLISION);

	pince_open();
	vTaskDelay(ms_to_tick(400));

	control_pince_dual(70, 0);
	do
	{
		control_goto_near(525, 175, 350);
		event = vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	}while(event & EVENT_US_COLLISION);

	do
	{
		control_goto(-525, 175);
		event = vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	}while(event & EVENT_US_COLLISION);
	
	do
	{
		control_goto_near(-900, 30, 160);
		event = vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	}while(event & EVENT_US_COLLISION);

	pince_close();
	vTaskDelay(ms_to_tick(400));

	control_pince_dual(1500, 0);
	vTaskDelay(ms_to_tick(300));

	do
	{
		control_goto_near(-875, 175, 160);
		event = vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	}while(event & EVENT_US_COLLISION);

	vTaskDelete(NULL);
}

