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
static void strat_ratissage_totem(int haut, int adverse);
static void strat_bouteille(int id);

int strat_module_init();

int strat_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(strat_task, "strat0", STRAT_STACK_SIZE, NULL, PRIORITY_TASK_STRATEGY, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_TEST;
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

	pince_open();

	strat_sortie();

	strat_ratissage_totem(1, 0);

	strat_bouteille(0);

//	strat_bouteille(1);

	vTaskDelete(NULL);
}

int start_wait_and_check_trajectory_result(enum trajectory_state wanted_state)
{
	uint32_t ev = vTaskWaitEvent(EVENT_TRAJECTORY_END, ms_to_tick(5000));
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

void strat_sortie()
{
	log(LOG_INFO, "sortie");
	// pas d'évitement par graph, on n'a pas le choix, il faut avancer
	do
	{
		trajectory_goto_near_xy(strat_dir * mm2fx(-900), mm2fx(775), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
		vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);
		vTaskDelay(ms_to_tick(50));
	}while(trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED);
}

static void strat_ratissage_totem(int haut, int adverse)
{
	log_format(LOG_INFO, "ratissage_totem haut = %d adv = %d", haut, adverse);
	if(haut)
	{
		haut = 1;
	}
	else
	{
		haut = -1;
	}

	if(adverse)
	{
		adverse = -strat_dir;
	}
	else
	{
		adverse = strat_dir;
	}

	do
	{
		trajectory_goto_near_xy(0, haut * mm2fx(400), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_GRAPH);
		vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);
		vTaskDelay(ms_to_tick(50));
	}while(trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED);

	do
	{
		trajectory_goto_near_xy(adverse * mm2fx(-600), haut * mm2fx(400), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_GRAPH);
		vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);
		vTaskDelay(ms_to_tick(50));
	}while(trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED);

	do
	{
		trajectory_goto_near_xy(adverse * mm2fx(-1250), 0, 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_GRAPH);
		vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);
		vTaskDelay(ms_to_tick(50));
	}while(trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED);
}

static void strat_bouteille(int id)
{
	log_format(LOG_INFO, "bouteille %d", id);

	// bouteille sur la ligne noir
	int32_t x = strat_dir * mm2fx(-860);
	if(id)
	{
		// seconde bouteille
		x = strat_dir * mm2fx(-383);
	}

	do
	{
		trajectory_goto_near(strat_dir * mm2fx(-860), mm2fx(-700), 1 << 24, TRAJECTORY_ANY_WAY, TRAJECTORY_AVOIDANCE_GRAPH);
	}while(start_wait_and_check_trajectory_result(TRAJECTORY_STATE_TARGET_REACHED) );

	do
	{
		trajectory_straight_to_wall();
	}while(start_wait_and_check_trajectory_result(TRAJECTORY_STATE_COLISION) );
}