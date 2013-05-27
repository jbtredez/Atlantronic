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
#include "control/trajectory.h"

#define STRAT_STACK_SIZE       300

static int strat_dir;

static void strat_task();
static void strat_sortie();
int strat_parcours_graph(enum trajectory_way);

int strat_module_init();

int strat_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(strat_task, "test_graph", STRAT_STACK_SIZE, NULL, PRIORITY_TASK_STRATEGY, &xHandle);

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

	strat_sortie();

	strat_parcours_graph(TRAJECTORY_FORWARD);
	
	vTaskDelete(NULL);
}

void strat_sortie()
{
	log(LOG_INFO, "sortie");
	// pas d'évitement par graph, on n'a pas le choix, il faut avancer
	
	do
	{
		trajectory_goto_near_xy(strat_dir * mm2fx(-750), mm2fx(775), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
		vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);
		vTaskDelay(ms_to_tick(50));
	}while(trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED);	
	
}

int strat_parcours_graph(enum trajectory_way way)
{
	static const uint32_t node_list[]={2, 0, 5, 4, 1, 0, 3, 6, 8, 10, 11, 14, 17, 15, 18, 19, 16, 12, 13, 9, 7, 2};
	static const uint32_t node_list_size = sizeof(node_list)/sizeof(node_list[0]);
	uint32_t i;

	for(i=0;i<node_list_size;i++)
	{
		trajectory_goto_graph_node(node_list[i], 0, way, TRAJECTORY_AVOIDANCE_GRAPH);
		vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);
		if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
		{
			return -1;
		}
	}

	return 0;
	
}
