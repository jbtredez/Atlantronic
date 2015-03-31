#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "kernel/systick.h"
#include "kernel/log.h"
#include "kernel/driver/usb.h"
#include "kernel/location/location.h"
#include "kernel/match.h"
#include "kernel/motion/trajectory.h"
#include "disco/wing.h"
#include "disco/elevator.h"
#include "disco/finger.h"
#include "disco/recalage.h"

#define STRAT_STACK_SIZE       300
#define FEET_APPROX_DIST       100

typedef struct
{
	const char* name;
	int (*action)(void* arg);
	void* arg;
	int errorCount;  //! 0 si réalisée, -1 si jamais tentée, nombre de fois ou l'action a été ratée sinon
}StratAction;

static void strat_task(void* arg);
static void strat_cmd(void* arg);

static int strat_start(void* arg);
static int strat_clap(void* arg);

static int strat_color;
StratAction strat_action[ ] =
{
		{ "start", strat_start, NULL, -1},
		{ "clap", strat_clap, NULL, -1},
};

int strat_module_init()
{
	portBASE_TYPE err = xTaskCreate(strat_task, "strat0", STRAT_STACK_SIZE, NULL, PRIORITY_TASK_STRATEGY, NULL);

	if(err != pdPASS)
	{
		return ERR_INIT_STRAT;
	}

	strat_color = match_get_color();

	usb_add_cmd(USB_CMD_STRAT, strat_cmd);

	return 0;
}

module_init(strat_module_init, INIT_STRATEGY);

static void strat_task(void* arg)
{
	(void) arg;

	match_wait_go();
	strat_color = match_get_color();

	// realisation des actions dans l'ordre au debut
	for(unsigned int i = 0; i < sizeof(strat_action)/sizeof(strat_action[0]); i++)
	{
		StratAction* a = &strat_action[i];
		log_format(LOG_INFO, "action %s", a->name);
		a->action(a->arg);
	}

	// TODO faire les actions manquantes jusqu'a la fin du match
	while(1)
	{
		vTaskDelay(100);
	}
}

static void strat_take_feet()
{
	finger_set_pos(FINGER_OPEN, FINGER_OPEN);
	vTaskDelay(500);
	elevator_set_position(0);
	vTaskDelay(800);
	finger_set_pos(FINGER_CLOSE, FINGER_CLOSE);
	vTaskDelay(200);
}

static int strat_start(void* /*arg*/)
{
	// prise ampoule
	finger_set_pos(FINGER_OPEN, FINGER_OPEN);
	vTaskDelay(500);
	elevator_set_position(50);
	vTaskDelay(500);
	finger_set_pos(FINGER_CLOSE, FINGER_CLOSE);
	vTaskDelay(500);

	// sortie case depart en marche arriere
	VectPlan dest(1000, 0, 0);
	trajectory_goto(dest.symetric(strat_color), WAY_BACKWARD, AVOIDANCE_STOP);
	trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000); // TODO verif cas erreur

	// premier pied
	elevator_set_position(100);
	trajectory_goto_near_xy(strat_color * 630, -355, FEET_APPROX_DIST, WAY_FORWARD, AVOIDANCE_STOP);
	trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000); // TODO verif cas erreur
	strat_take_feet();

	// second pied
	elevator_set_position(100);
	trajectory_goto_near_xy(strat_color * 200, -400, FEET_APPROX_DIST, WAY_FORWARD, AVOIDANCE_STOP);
	trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000); // TODO verif cas erreur
	strat_take_feet();

	// pied 3
	elevator_set_position(100);
	trajectory_goto_near_xy(strat_color * 400, -750, FEET_APPROX_DIST, WAY_FORWARD, AVOIDANCE_STOP);
	trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000); // TODO verif cas erreur
	strat_take_feet();

	return 0;
}

static int strat_clap(void* /*arg*/)
{
	/*VectPlan nextToClap(1200, -750, 0);
	log_format(LOG_INFO, "color %d", strat_color);
	trajectory_goto(nextToClap.symetric(strat_color), WAY_BACKWARD, AVOIDANCE_STOP);
	wing_set_position(strat_color!=1, WING_PARK, WING_OPEN);
	trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000); // TODO verif cas erreur
*/
	return 0;
}

static void strat_cmd(void* /*arg*/)
{
	// TODO test des actions unitairement
}
