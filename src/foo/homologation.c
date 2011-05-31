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
	control_goto(0, -700);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	control_goto_near(0, 0, 160);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	pince_close();
	vTaskDelay(ms_to_tick(300));
	control_pince_dual(1500, 0);
	vTaskDelay(ms_to_tick(300));
	control_goto_near(175, -175, 160);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	pince_open();

	vTaskDelete(NULL);
}

