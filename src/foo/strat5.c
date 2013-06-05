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
#include "control/trajectory.h"
#include "foo/arm.h"
#include "foo/strat.h"

#define STRAT_STACK_SIZE       300

static int strat_dir;
static int strat_bouteille1_ok;
static int strat_bouteille2_ok;
static int strat_totem1_high_ok;
static int strat_totem1_low_ok;
static int strat_totem2_high_ok;
static int strat_totem2_low_ok;
static int strat_steal_ok;

static void strat_task();
static int strat_bouteille(int id);
static int strat_cale(int high);
static int strat_totem(int high);
static int strat_cleanup_bottle_way();
static int strat_oponent_totem(int high);
static int strat_steal_coins_inside();

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

	gpio_wait_go();

	if(getcolor() != COLOR_BLUE)
	{
		strat_dir = -1;
	}

	strat_bouteille1_ok = -1;
	strat_bouteille2_ok = -1;
	strat_totem1_high_ok = -1;
	strat_totem1_low_ok = -1;
	strat_totem2_high_ok = -1;
	strat_totem2_low_ok = -1;
	int cleanu_bottle_ok = -1;
	strat_steal_ok = -1;

	while(1)
	{
		if( strat_totem1_high_ok < 0)
		{
			strat_totem1_high_ok = strat_totem(1);
		}

		if( cleanu_bottle_ok < 0)
		{
			strat_cleanup_bottle_way();
			cleanu_bottle_ok = 0;
		}

		if(strat_bouteille1_ok < 0)
		{
			strat_bouteille1_ok = strat_bouteille(0);
		}

		if( strat_totem1_low_ok < 0)
		{
			strat_totem1_low_ok = strat_totem(-1);
		}

		if( strat_bouteille2_ok < 0)
		{
			strat_bouteille2_ok = strat_bouteille(1);
		}

		if( strat_totem2_low_ok < 0)
		{
			strat_oponent_totem(-1);
			strat_totem2_low_ok = 0;
		}

		/*if( strat_totem2_high_ok < 0)
		{
			strat_oponent_totem(1);
			strat_totem2_high_ok = 0;
		}*/

		if(strat_steal_ok<0)
		{
			strat_steal_coins_inside();

			// prépositionnement
			trajectory_goto_near_xy(strat_dir * mm2fx(-760), mm2fx(320), 0, TRAJECTORY_ANY_WAY, TRAJECTORY_AVOIDANCE_GRAPH);
			vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

			strat_cale(1);

			strat_steal_ok = 0;
		}

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
		y = mm2fx(130);
	}

	if(strat_dir * high == 1)
	{
		pince_set_position(PINCE_OPEN, PINCE_MIDDLE);
	}
	else
	{
		pince_set_position(PINCE_MIDDLE, PINCE_OPEN);
	}
	trajectory_goto_near_xy( strat_dir * mm2fx(-1110), y, 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	vTaskDelay(ms_to_tick(500));

	// on recule. On se le tente plusieurs fois pour ne pas fermer les pinces dans la cale
	int i = 0;
	do
	{
		trajectory_goto_near_xy( strat_dir * mm2fx(-762), high*mm2fx(312), 0, TRAJECTORY_BACKWARD, TRAJECTORY_AVOIDANCE_STOP);
		vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);
		vTaskDelay(ms_to_tick(100));
		if( i > 3000)
		{
			trajectory_straight(-300<<16);
			vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);
			goto end;
		}
	}
	while(trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED);

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

static int strat_cleanup_bottle_way()
{
	int res = 0;

	pince_set_position(PINCE_MIDDLE, PINCE_MIDDLE);

	trajectory_goto_near_xy( strat_dir * mm2fx(-860), mm2fx(-600), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);
	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		res = -1;
		trajectory_goto_near_xy( strat_dir * mm2fx(-860), mm2fx(0), 0, TRAJECTORY_BACKWARD, TRAJECTORY_AVOIDANCE_STOP);
		vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);
		strat_cale(-1);
		goto end;
	}


	// calage contre le mur avec les pinces
	trajectory_goto_near_xy( strat_dir * mm2fx(-860), mm2fx(-630), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	trajectory_straight(mm2fx(-40));
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	pince_set_position(PINCE_STRAT, PINCE_STRAT);
	vTaskDelay(ms_to_tick(500));

	trajectory_rotate(strat_dir * 1<<25);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

end:
	return res;
}

static int strat_totem(int high)
{
	int res = 0;

	log_format(LOG_INFO, "high %d", high);

	if(strat_dir*high == 1)
	{
		pince_set_position(PINCE_MIDDLE, PINCE_OPEN);
	}
	else
	{
		pince_set_position(PINCE_OPEN, PINCE_MIDDLE);
	}

	// on se met en face du palmier
	int32_t y;
	if( high == 1)
	{
		y = mm2fx(760);
	}
	else
	{
		y = mm2fx(-600);
	}

	int i = 0;
	do
	{
		trajectory_goto_near_xy( strat_dir * mm2fx(0), y, 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
		i++;
	}while(start_wait_and_check_trajectory_result(TRAJECTORY_STATE_TARGET_REACHED) && i < 10);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		res = -1;
		trajectory_goto_near_xy( strat_dir * mm2fx(-600), mm2fx(450), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
		vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);
		strat_cale(high);
		return -1;
//		goto end;
	}

	// on va vers le palmier
	trajectory_set_detection_dist_min(PARAM_RIGHT_CORNER_X + 150);
	trajectory_goto_near_xy( strat_dir * mm2fx(0), high * mm2fx(400), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskDelay(ms_to_tick(2000));

	if(strat_dir*high == 1)
	{
		pince_set_position(PINCE_STRAT, PINCE_OPEN);
	}
	else
	{
		pince_set_position(PINCE_OPEN, PINCE_STRAT);
	}

	vTaskDelay(ms_to_tick(1500));

	if(high == 1)
	{
		arm_hook_goto(300<<16, 90<<16, 0, 90<<16, 0, 1);
	}
	else
	{
		arm_hook_goto(0, -90<<16, 300<<16, -90<<16, 0, 1);
	}

	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	// TODO desactiver sur une zone
	//trajectory_set_detection_dist_min(PARAM_RIGHT_CORNER_X + 300<<16);
	trajectory_disable_hokuyo();

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		res = -1;
//		goto end;
	}

	trajectory_goto_near_xy( strat_dir * mm2fx(-600), high * mm2fx(365), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskDelay(ms_to_tick(500));
	if(strat_dir*high == 1)
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
		strat_cale(high);
		res = -1;
//		goto end;
	}

	arm_close();
	res = strat_cale(high);

/*end:
	arm_close();*/
	trajectory_enable_hokuyo();
	trajectory_set_detection_dist_min(PARAM_RIGHT_CORNER_X);
	return res;
}

static int strat_oponent_totem(int high)
{
	int res = 0;

	log_format(LOG_INFO, "high %d", high);

	// on se met en face
	trajectory_goto_near_xy( strat_dir * mm2fx(600), high * mm2fx(650), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
	start_wait_and_check_trajectory_result(TRAJECTORY_STATE_TARGET_REACHED);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		res = -1;
		goto end;
	}

	// on va vers le palmier
	trajectory_goto_near_xy( strat_dir * mm2fx(750), high * mm2fx(400), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskDelay(ms_to_tick(2000));

	if(-strat_dir*high == 1)
	{
		pince_set_position(PINCE_CLOSE, PINCE_OPEN);
	}
	else
	{
		pince_set_position(PINCE_OPEN, PINCE_CLOSE);
	}

	vTaskDelay(ms_to_tick(1500));
/*
	if(high == 1)
	{
		arm_hook_goto(300<<16, 90<<16, 0, 90<<16, 0, 1);
	}
	else
	{
		arm_hook_goto(0, -90<<16, 300<<16, -90<<16, 0, 1);
		vTaskDelay(ms_to_tick(500));
	}
*/
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);
	vTaskDelay(ms_to_tick(300));

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		res = -1;
		goto end;
	}

	trajectory_goto_near_xy( strat_dir * mm2fx(-600), high * mm2fx(360), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskDelay(ms_to_tick(300));
	if(strat_dir*high == 1)
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
		strat_cale(high);
		res = -1;
		goto end;
	}

	arm_close();
	res = strat_cale(high);

end:
	arm_close();
	trajectory_set_detection_dist_min(PARAM_RIGHT_CORNER_X);
	return res;
}

static int strat_steal_coins_inside()
{
	log_format(LOG_INFO, " En mode voleur (inside) ");
	int32_t alpha;
	
	if(strat_dir == 1)
	{
		alpha = 0;
	}
	else
	{
		alpha = 1 << 25;
	}
	
	// prépositionnement
	trajectory_goto_near(strat_dir * mm2fx(900), mm2fx(170), alpha, 0, TRAJECTORY_ANY_WAY, TRAJECTORY_AVOIDANCE_GRAPH);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);
	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		return -1;
	}

	// on ouvre les pinces
	pince_set_position(PINCE_OPEN,PINCE_OPEN);
	vTaskDelay(ms_to_tick(500));

	trajectory_disable_static_check ();
	control_set_max_speed( (1 << 16) / 3, 1 << 16);
	
	// on avance
	trajectory_straight (mm2fx(250));
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);
	trajectory_enable_static_check();

	// on ferme les pinces
	pince_set_position(PINCE_STRAT,PINCE_STRAT);
	vTaskDelay(ms_to_tick(500));

	if(strat_dir == 1)
	{
		alpha = 1 << 25;
	}
	else
	{
		alpha = 0;
	}	

	// on sort
	do
	{
		trajectory_goto_near(strat_dir * mm2fx(900), mm2fx(170), alpha, 0, TRAJECTORY_ANY_WAY, TRAJECTORY_AVOIDANCE_STOP);
		vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);
	}
	while(trajectory_get_state() == TRAJECTORY_STATE_TARGET_REACHED);

	return 1;
}	
#if 0
static int strat_return_coins_inside()
{
	log_format(LOG_INFO, " En mode retour tresor ");
	int32_t alpha;
	
	if(strat_dir == 1)
	{
		alpha = 1 << 25;
	}
	else
	{
		alpha = 0;
	}
	
	// prépositionnement
	trajectory_goto_near(strat_dir * mm2fx(-900), mm2fx(150), alpha, 0, TRAJECTORY_ANY_WAY, TRAJECTORY_AVOIDANCE_STOP);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);
	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		return -1;
	}

	trajectory_disable_static_check ();
	control_set_max_speed( (1 << 16) / 3, 1 << 16);

	// on ouvre les pinces
	pince_set_position(PINCE_OPEN,PINCE_OPEN);
	vTaskDelay(ms_to_tick(500));
	
	// on avance
	trajectory_straight (mm2fx(250));
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);
	control_set_max_speed( (1 << 16), 1 << 16);
	trajectory_enable_static_check();

		
	if(strat_dir == 1)
	{
		alpha = 0;
	}
	else
	{
		alpha = 1 << 25;
	}	

	// on sort
	trajectory_goto_near(strat_dir * mm2fx(-900), mm2fx(170), alpha, 0, TRAJECTORY_BACKWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);
	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		return -1;
	}

	// on ferme les pinces
	pince_set_position(PINCE_CLOSE,PINCE_CLOSE);
	vTaskDelay(ms_to_tick(500));

	return 1;
}
#endif
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

	trajectory_goto_near(x, mm2fx(-550), 1 << 24, 0, TRAJECTORY_ANY_WAY, TRAJECTORY_AVOIDANCE_STOP);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		trajectory_goto_near_xy( strat_dir * mm2fx(-860), mm2fx(0), 0, TRAJECTORY_BACKWARD, TRAJECTORY_AVOIDANCE_STOP);
		vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);
		strat_cale(-1);
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
