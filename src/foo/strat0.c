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

enum totem_pos
{
	TOTEM_POS_HIGH,
	TOTEM_POS_LOW,
	TOTEM_POS_OP_HIGH,
	TOTEM_POS_OP_LOW,
};

static int strat_dir;

static void strat_task();
static void strat_sortie();
static int strat_ratissage_totem(enum totem_pos pos);
static int strat_bouteille(int id);
//!< on vide le totem avec le robot orienté selon l'axe y
int strat_sortie_totem_y(int totem);

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

//	trajectory_disable_hokuyo();

	int res = strat_sortie_totem_y(strat_dir);

//	strat_ratissage_totem(TOTEM_POS_OP_HIGH);
//	strat_ratissage_totem(TOTEM_POS_HIGH);

	strat_bouteille(0);

	strat_bouteille(1);
#if 0
	strat_bouteille(0);

	int i = 0;
	do
	{
		trajectory_goto_near_xy( strat_dir * mm2fx(-600), mm2fx(-400), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
		vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);
		vTaskDelay(ms_to_tick(50));
		i++;
	}while(trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED && i < 3);

	strat_bouteille(1);
#endif

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

int strat_sortie_totem_y(int totem)
{
	int res = 0;
	log_format(LOG_INFO, "strat_sortie_totem_y %d", totem);

	// on se met en face du totem
	arm_set_tool_way(-1);

	trajectory_goto_near_xy( - totem * mm2fx(400), mm2fx(775), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		res = -1;
		goto end;
	}

 	pince_set_position(PINCE_OPEN, PINCE_OPEN);

 	// on va vers le totem
	trajectory_goto_near_xy( -totem * mm2fx(400), mm2fx(420), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskDelay(ms_to_tick(500));
	pince_set_position(PINCE_OPEN, PINCE_OPEN);

	// bras visible
	trajectory_set_detection_dist_min(PARAM_RIGHT_CORNER_X + ARM_L1 + ARM_L2);
	vTaskDelay(ms_to_tick(500));
	arm_goto_xyz(300<<16, 0, 110<<16, 1);
	vTaskDelay(ms_to_tick(200));
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

 	arm_goto_xyz(200<<16, 0, 150<<16, 1);
 	vTaskDelay(ms_to_tick(500));
	trajectory_set_detection_dist_min(PARAM_RIGHT_CORNER_X);
	arm_bridge_off();

	// on s'éloigne du totem
	trajectory_goto_near_xy( -totem * mm2fx(400), mm2fx(480), 0, TRAJECTORY_BACKWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		res = -1;
		goto end;
	}

	pince_set_position(PINCE_MIDDLE, PINCE_MIDDLE);

	// on part vers la zone de dépot
	trajectory_goto_near_xy( strat_dir * mm2fx(-1110), mm2fx(170), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		res = -1;
		goto end;
	}

	// on recule
	trajectory_goto_near_xy( strat_dir * mm2fx(-762), mm2fx(312), 0, TRAJECTORY_BACKWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		res = -1;
		goto end;
	}

end:
	pince_set_position(PINCE_CLOSE, PINCE_CLOSE);

	return res;
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

static int strat_ratissage_totem(enum totem_pos pos)
{
	int dir;
	int high;

	log_format(LOG_INFO, "ratissage_totem %d", pos);
	switch(pos)
	{
		default:
		case TOTEM_POS_HIGH:
			high = 1;
			dir = strat_dir;
			break;
		case TOTEM_POS_LOW:
			high = -1;
			dir = strat_dir;
			break;
		case TOTEM_POS_OP_HIGH:
			high = 1;
			dir = -strat_dir;
			break;
		case TOTEM_POS_OP_LOW:
			high = -1;
			dir = -strat_dir;
			break;
	}

	// on anticipe sur le servo
	arm_set_tool_way(0);

	trajectory_goto_near_xy(0, high * mm2fx(310), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		return -1;
	}

	// TODO voir les pinces
	// pince_set_position(PINCE_CLOSE,PINCE_CLOSE);
	int32_t alpha =  -0.02f*(1<<26);
	if( dir == 1)
	{
		alpha = (1<<25) - alpha;
	}

	trajectory_goto_near(dir * mm2fx(-75), high * mm2fx(310), alpha, 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		return -1;
	}

	// prise du lingo
	arm_ventouse_goto(3000 << 16, 140<<16, 0, 140<<16, 120<<16, dir);
	vTaskDelay(ms_to_tick(1000));
	arm_bridge_on();
	arm_ventouse_goto( 3000 << 16, 65<<16, 0, 65<<16, 120<<16, dir);
	vTaskDelay(ms_to_tick(1000));

	// retour vers le centre du robot
	arm_goto_xyz(200<<16, 0, 120<<16, ARM_CMD_XYZ_LOC);
	vTaskDelay(ms_to_tick(500));

	// on débute la marche arrière
	trajectory_goto_near_xy(0, mm2fx(310), 0, TRAJECTORY_BACKWARD, TRAJECTORY_AVOIDANCE_STOP);
	arm_bridge_off();
	arm_set_tool_way(-dir);
	vTaskDelay(ms_to_tick(500));
	// préparation de l'outil à la bonne hauteur
	arm_goto_xyz(200<<16, 0, 220<<16, ARM_CMD_XYZ_LOC);
	vTaskDelay(ms_to_tick(1500));

	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		return -1;
	}

	arm_hook_goto( 3000 << 16, 110<<16, 0, 110<<16, 220<<16, -dir);
	vTaskDelay(ms_to_tick(2000));

	trajectory_goto_near_xy(strat_dir * mm2fx(-760), mm2fx(310), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		return -1;
	}

	trajectory_goto_near_xy(strat_dir * mm2fx(-1250), mm2fx(150), 0, TRAJECTORY_FORWARD, TRAJECTORY_AVOIDANCE_STOP);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		return -1;
	}

	return 0;
}

static int strat_bouteille(int id)
{
	log_format(LOG_INFO, "bouteille %d", id);

	// bouteille sur la ligne noir
	int32_t x = strat_dir * mm2fx(-860);
	if(id)
	{
		// seconde bouteille
		x = strat_dir * mm2fx(383);
	}

	trajectory_goto_near(x, mm2fx(-400), 1 << 24, 0, TRAJECTORY_ANY_WAY, TRAJECTORY_AVOIDANCE_GRAPH);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_REACHED)
	{
		return -1;
	}

	control_disable_sick();
	trajectory_disable_static_check();

	trajectory_goto_near_xy(x, mm2fx(-1000), 0, TRAJECTORY_BACKWARD, TRAJECTORY_AVOIDANCE_GRAPH);
	vTaskWaitEvent(EVENT_TRAJECTORY_END, portMAX_DELAY);

	if( trajectory_get_state() != TRAJECTORY_STATE_TARGET_NOT_REACHED && trajectory_get_state() != TRAJECTORY_STATE_COLISION)
	{
		return -1;
	}

	vTaskDelay(ms_to_tick(200));
	trajectory_straight( mm2fx(300));
	start_wait_and_check_trajectory_result(TRAJECTORY_STATE_TARGET_REACHED);

	trajectory_enable_static_check();
	control_enable_sick();

	return 0;
}
