#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "kernel/systick.h"
#include "kernel/event.h"
#include "kernel/robot_parameters.h"
#include "kernel/log.h"
#include "gpio.h"
#include "location/location.h"
#include "kernel/rcc.h"
#include "pince.h"
#include "recalage.h"
#include "us.h"
#include "control/trajectory.h"

#define STRAT_STACK_SIZE       300

static int strat_dir;

static void strat_task();
static void strat_sortie();

int strat_module_init();

int strat_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(strat_task, "strat0", STRAT_STACK_SIZE, NULL, PRIORITY_TASK_STRATEGY, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_STRAT;
	}

	strat_dir = 1;

	return 0;
}

module_init(strat_module_init, INIT_STRATEGY);

static void strat_task()
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

	if(getcolor() != COLOR_BLUE)
	{
		strat_dir = -1;
	}

	trajectory_disable_hokuyo();

	strat_sortie();

	trajectory_goto_graph_node(0, 0, TRAJECTORY_ANY_WAY);

	vTaskDelete(NULL);
}

void strat_sortie()
{
	log(LOG_INFO, "sortie");
	// pas d'évitement par graph, on n'a pas le choix, il faut avancer
	do
	{
		trajectory_goto_near_xy(strat_dir * mm2fx(0), mm2fx(775), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
		vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);
		vTaskDelay(ms_to_tick(50));
	}while(trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED);
}
