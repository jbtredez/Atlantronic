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

static int strat_dir;
static int strat_bouteille1_ok;
static int strat_bouteille2_ok;
static int strat_totem2_high;
static int strat_totem1_low;
static int strat_middle_low_ok;

static void strat_task();
static int strat_bouteille(int id);
//!< on vide le totem avec le robot orienté selon l'axe y
static int strat_sortie_totem_y(int totem, int high);
static int strat_middle_low();
static int strat_cale(int high);
static int strat_totem(int totem, int high);

static void strat_cmd(void* arg);

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

	usb_add_cmd(USB_CMD_STRAT, strat_cmd);

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

	strat_bouteille1_ok = -1;
	strat_bouteille2_ok = -1;
	strat_totem2_high = -1;
	strat_totem1_low = -1;
	strat_middle_low_ok = -1;

	if(strat_dir == 1)
	{
		pince_set_position(PINCE_MIDDLE, PINCE_OPEN);
	}
	else
	{
		pince_set_position(PINCE_OPEN, PINCE_MIDDLE);
	}

//	int res = strat_sortie_totem_y(strat_dir, 1);

//	while(strat_bouteille1_ok < 0 && strat_totem1_low < 0 && strat_bouteille2_ok < 0)
	{
		strat_totem(1,1);
/*		if(strat_bouteille1_ok < 0)
		{
			strat_bouteille1_ok = strat_bouteille(0);
		}

		if( strat_totem1_low < 0)
		{
			strat_totem1_low = strat_sortie_totem_y(strat_dir, -1);
		}

		if( strat_bouteille2_ok < 0)
		{
			strat_bouteille2_ok = strat_bouteille(1);
		}
*/
		vTaskDelay(ms_to_tick(100));
	}
/*
	struct fx_vect_pos pos = location_get_position();

	if( pos.y > 0)
	{
		strat_cale(1);
	}
	else
	{
		strat_cale(-1);
	}
*/
	vTaskDelete(NULL);
}

static void strat_cmd(void* arg)
{
	struct strat_cmd_arg* cmd = (struct strat_cmd_arg*) arg;

	switch(cmd->type)
	{
		case STRAT_BOUTEILLE:
			strat_bouteille(cmd->arg1);
			break;
		default:
			break;
	}
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

static int strat_cale(int high)
{
	int res;
	int32_t y;

	// on part vers la zone de dépot
	trajectory_disable_static_check();
	if(high == 1)
	{
		y = mm2fx(170);
	}
	else
	{
		y = 0;
	}

	trajectory_goto_near_xy( strat_dir * mm2fx(-1110), y, 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		res = -1;
		goto end;
	}

	pince_set_position(PINCE_MIDDLE, PINCE_MIDDLE);
	vTaskDelay(ms_to_tick(500));

	// on recule. On se le tente plusieurs fois pour ne pas fermer les pinces dans la cale
	int i = 0;
	do
	{
		trajectory_goto_near_xy( strat_dir * mm2fx(-762), high*mm2fx(312), 0, TRAJECTORY_BACKWARD, TRAJECTORY_AVOIDANCE_STOP);
		vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);
		i++;
		vTaskDelay(ms_to_tick(100));
	}
	while(trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED && i < 5);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		res = -1;
		goto end;
	}

end:
	pince_set_position(PINCE_CLOSE, PINCE_CLOSE);
	trajectory_enable_static_check();

	return res;
}

static int strat_totem(int totem, int high)
{
	int res = 0;

	log_format(LOG_INFO, "strat_totem %d high %d", totem, high);

	// on se met en face du totem
	int32_t y;
	if( high == 1)
	{
		y = mm2fx(775);
	}
	else
	{
		y = mm2fx(-600);
	}

	trajectory_goto_near_xy( totem * mm2fx(0), y, 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		res = -1;
		goto end;
	}

	// on va vers le palmier
	trajectory_set_detection_dist_min(PARAM_RIGHT_CORNER_X + 150);
	trajectory_goto_near_xy( totem * mm2fx(0), mm2fx(380), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskDelay(ms_to_tick(1000));
	if(strat_dir == 1)
	{
		pince_set_position(PINCE_CLOSE, PINCE_OPEN);
	}
	else
	{
		pince_set_position(PINCE_OPEN, PINCE_CLOSE);
	}
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		res = -1;
		goto end;
	}

	trajectory_goto_near_xy( strat_dir * mm2fx(-600), mm2fx(360), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskDelay(ms_to_tick(300));
	if(strat_dir == 1)
	{
		pince_set_position(PINCE_OPEN, PINCE_MIDDLE);
	}
	else
	{
		pince_set_position(PINCE_MIDDLE, PINCE_OPEN);
	}

	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		res = -1;
		goto end;
	}

	res = strat_cale(high);

end:
	trajectory_set_detection_dist_min(PARAM_RIGHT_CORNER_X);
	return res;
}

static int strat_sortie_totem_y(int totem, int high)
{
	int res = 0;
	log_format(LOG_INFO, "strat_sortie_totem_y %d", totem);

	// on se met en face du totem
	arm_set_tool_way(-1);

	int32_t y;
	if( high == 1)
	{
		y = mm2fx(775);
	}
	else
	{
		y = mm2fx(-600);
	}

	trajectory_goto_near_xy( - totem * mm2fx(400), y, 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		res = -1;
		goto end;
	}

 	pince_set_position(PINCE_OPEN, PINCE_OPEN);

 	// on va vers le totem
	trajectory_goto_near_xy( -totem * mm2fx(400), high*mm2fx(410), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskDelay(ms_to_tick(500));
	pince_set_position(PINCE_OPEN, PINCE_OPEN);

	// bras visible
	trajectory_set_detection_dist_min(PARAM_RIGHT_CORNER_X + ARM_L1 + ARM_L2);
	vTaskDelay(ms_to_tick(500));
	arm_goto_xyz(300<<16, 0, 110<<16, 1);
	vTaskDelay(ms_to_tick(500));
	arm_bridge_on();
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		res = -1;
		goto end;
	}

 	arm_goto_xyz(200<<16, 0, 110<<16, 1);
 	vTaskDelay(ms_to_tick(1000));

	if(strat_dir == -1)
	{
		pince_set_position(PINCE_OPEN, PINCE_MIDDLE);
	}
	else
	{
		pince_set_position(PINCE_MIDDLE, PINCE_OPEN );
	}

	arm_goto_xyz(200<<16, 0, 110<<16, 1);
	vTaskDelay(ms_to_tick(1000));
	arm_bridge_off();

 	arm_goto_xyz(200<<16, 0, 150<<16, 1);
 	vTaskDelay(ms_to_tick(500));
	trajectory_set_detection_dist_min(PARAM_RIGHT_CORNER_X);

	// on s'éloigne du totem
	trajectory_goto_near_xy( -totem * mm2fx(400), high*mm2fx(500), 0, TRAJECTORY_BACKWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		res = -1;
		goto end;
	}

	pince_set_position(PINCE_MIDDLE, PINCE_MIDDLE);

	// on part vers la zone de dépot
	res = strat_cale(high);

end:
 	arm_goto_xyz(200<<16, 0, 150<<16, 1);
	trajectory_set_detection_dist_min(PARAM_RIGHT_CORNER_X);
	arm_bridge_off();
	pince_set_position(PINCE_CLOSE, PINCE_CLOSE);
	trajectory_enable_static_check();

	return res;
}

//!< 4cd + lingo du milieu
static int strat_middle_low()
{
	int res = 0;

	trajectory_goto_near(strat_dir * mm2fx(-400), mm2fx(-700), 1 << 24, 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_GRAPH);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		res = -1;
		goto end;
	}

	pince_set_position(PINCE_MIDDLE, PINCE_MIDDLE);

	trajectory_goto_near(strat_dir * mm2fx(200), mm2fx(-700), 1 << 24, 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		res = -1;
		goto end;
	}

	pince_set_position(PINCE_CLOSE, PINCE_CLOSE);

	trajectory_goto_near(strat_dir * mm2fx(200), mm2fx(-400), 1 << 24, 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		res = -1;
		goto end;
	}

	if(strat_dir == 1)
	{
		pince_set_position(PINCE_CLOSE, PINCE_OPEN);
	}
	else
	{
		pince_set_position(PINCE_OPEN, PINCE_CLOSE);
	}

	trajectory_goto_near(strat_dir * mm2fx(-400), mm2fx(-400), 1 << 24, 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		res = -1;
		goto end;
	}

	pince_set_position(PINCE_MIDDLE, PINCE_MIDDLE);

	strat_cale(-1);

end:
	pince_set_position(PINCE_CLOSE, PINCE_CLOSE);
	return res;
}

static int strat_bouteille(int id)
{
	int res = 0;

	log_format(LOG_INFO, "bouteille %d", id);

	// bouteille sur la ligne noir
	int32_t x = strat_dir * mm2fx(-860);
	if(id)
	{
		// seconde bouteille
		x = strat_dir * mm2fx(383);
	}

	trajectory_goto_near(x, mm2fx(-550), 1 << 24, 0, TRAJECTORY_ANY_WAY, TRAJECTORY_AVOIDANCE_GRAPH);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		return -1;
	}

	control_disable_sick();
	trajectory_disable_static_check();

	trajectory_goto_near_xy(x, mm2fx(-1000), 0, TRAJECTORY_BACKWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_NOT_REACHED && trajectory_get_state() != TRAJECTORY_STATE_COLISION)
	{
		res = -1;
		goto end;
	}

	vTaskDelay(ms_to_tick(200));
	trajectory_straight( mm2fx(300));
	start_wait_and_check_trajectory_result(TRAJECTORY_STATE_TARGET_REACHED);

end:
	trajectory_enable_static_check();
	control_enable_sick();

	return res;
}
