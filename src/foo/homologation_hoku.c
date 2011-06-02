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
#include "avoidance/macro_fonction.h"

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
	avoidance_init_table();
	
	return 0;
}

module_init(homologation_module_init, INIT_STRATEGY);

static void homologation_task()
{

#if 0
	while(getGo() == 0)
	{
		if( getRecalage() )
		{
			recalage();
			resetRecalage();
		}
		vTaskDelay(ms_to_tick(50));
	}
#endif
	vTaskWaitEvent(EVENT_GO, portMAX_DELAY);
	//on attend le pre-scan du départ de l hokuyo
	vTaskWaitEvent(EVENT_GO, EVENT_HOKUYO_READY);
	

#if 0
	 //go go on sort de notre grotte
	pince_open();
	control_pince_dual(70, 0);
	control_goto(-875, -850);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	
 	control_goto_near(0, 0, 160);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	
	//control_rotate();
#endif 	
	while(1) 
	  vTaskDelay(ms_to_tick(1000));
	
// 	pince_close();
// 	vTaskDelay(ms_to_tick(300));
// 	control_pince_dual(1500, 0);
// 	vTaskDelay(ms_to_tick(300));
// 	control_goto_near(175, -175, 160);
// 	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
// 	pince_open();

	vTaskDelete(NULL);
}

