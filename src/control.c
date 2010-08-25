//! @file control.c
//! @brief Asservissement
//! @author Jean-Baptiste Trédez

#include "FreeRTOS.h"
#include "task.h"
#include "module.h"
#include "queue.h"
#include "control.h"
#include "priority.h"
#include "log.h"
#include "odometry.h"
#include "vect_pos.h"

//! @todo réglage au pif
#define CONTROL_STACK_SIZE       10

//! période de la tache de propulsion en tick ("fréquence" de l'asservissement)
#define CONTROL_TICK_PERIOD        5

static void control_task(void *);

//////! @todo asservissement - choix ECN/ESEO
static int32_t state;
static struct vect_pos target;
//////

static int control_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(control_task, (const signed char *) "control", CONTROL_STACK_SIZE, NULL, PRIORITY_TASK_CONTROL, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_CONTROL;
	}

	target.x = 0;
	target.y = 0;
	target.alpha = 0;
	state = READY;

	return 0;
}

module_init(control_module_init, INIT_CONTROL);

static void control_task(void* arg)
{
	(void) arg;

	portTickType xLastWakeTime = xTaskGetTickCount();
	struct vect_pos pos;

	while(1)
	{
		odometry_update();
		pos = odometry_get_position();

		//////! @todo asservissement - choix ECN/ESEO
		switch(state)
		{
			default:
			case READY:
				break;
		}
		//////

		vTaskDelayUntil(&xLastWakeTime, CONTROL_TICK_PERIOD);
	}
}
