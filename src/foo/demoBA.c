#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "kernel/systick.h"
#include "kernel/event.h"
#include "kernel/robot_parameters.h"
#include "kernel/log.h"
#include "kernel/driver/usb.h"
#include "gpio.h"
#include "location/location.h"
#include "kernel/rcc.h"
#include "pince.h"
#include "recalage.h"
#include "us.h"
#include "control/trajectory.h"
#include "foo/arm.h"
#include "foo/strat.h"

#define STRAT_STACK_SIZE       300

static void strat_task();
int strat_module_init();
static int start_wait_and_check_trajectory_result(enum trajectory_state wanted_state);

int strat_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(strat_task, "strat0", STRAT_STACK_SIZE, NULL, PRIORITY_TASK_STRATEGY, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_STRAT;
	}

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

	pince_set_position(PINCE_CLOSE, PINCE_CLOSE);

	while(1)
	{
		trajectory_goto_near_xy( mm2fx(-1000), mm2fx(-1000), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_GRAPH);
		start_wait_and_check_trajectory_result(TRAJECTORY_STATE_TARGET_REACHED);

		trajectory_goto_near_xy( mm2fx(1000), mm2fx(-1000), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_GRAPH);
		start_wait_and_check_trajectory_result(TRAJECTORY_STATE_TARGET_REACHED);

		trajectory_goto_near_xy( mm2fx(1000), mm2fx(1000), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_GRAPH);
		start_wait_and_check_trajectory_result(TRAJECTORY_STATE_TARGET_REACHED);

		trajectory_goto_near_xy( mm2fx(-1000), mm2fx(1000), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_GRAPH);
		start_wait_and_check_trajectory_result(TRAJECTORY_STATE_TARGET_REACHED);

		trajectory_goto_near_xy( mm2fx(-1000), mm2fx(-1000), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_GRAPH);
		start_wait_and_check_trajectory_result(TRAJECTORY_STATE_TARGET_REACHED);

		trajectory_goto_near_xy( mm2fx(1000), mm2fx(1000), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_GRAPH);
		start_wait_and_check_trajectory_result(TRAJECTORY_STATE_TARGET_REACHED);

		trajectory_goto_near_xy( mm2fx(-1000), mm2fx(1000), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_GRAPH);
		start_wait_and_check_trajectory_result(TRAJECTORY_STATE_TARGET_REACHED);

		trajectory_goto_near_xy( mm2fx(1000), mm2fx(-1000), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_GRAPH);
		start_wait_and_check_trajectory_result(TRAJECTORY_STATE_TARGET_REACHED);

		vTaskDelay(ms_to_tick(100));
	}

	vTaskDelete(NULL);
}

static int start_wait_and_check_trajectory_result(enum trajectory_state wanted_state)
{
	uint32_t ev = vTaskWaitEvent(EVENT_TRAJECTORY_END, ms_to_tick(8000));
	if( !( ev & EVENT_TRAJECTORY_END) )
	{
		log(LOG_ERROR, "timeout");
		return -1;
	}

	enum trajectory_state state = trajectory_get_state();
	if(state != wanted_state)
	{
		// au cas ou, pour mettre la fonction dans un while
		vTaskDelay(ms_to_tick(50));
		log(LOG_ERROR, "incorrect state");
		return -1;
	}

	return 0;
}
