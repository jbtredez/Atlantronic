#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "kernel/systick.h"
#include "kernel/log.h"
#include "kernel/driver/usb.h"
#include "kernel/location/location.h"
#include "kernel/match.h"
#include "kernel/motion/trajectory.h"
#include "disco/recalage.h"

#define STRAT_STACK_SIZE       300

typedef struct
{
	const char* name;
	int (*action)(void* arg);
	void* arg;
	int errorCount;  //! 0 si réalisée, -1 si jamais tentée, nombre de fois ou l'action a été ratée sinon
}StratAction;

static void strat_task(void* arg);
static void strat_cmd(void* arg);
static int strat_clap(void* arg);

static int strat_color;
StratAction strat_action[ ] =
{
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

static int strat_clap(void* arg)
{
	(void) arg;
	// TODO
	trajectory_goto(VectPlan(0,0,0), WAY_ANY, AVOIDANCE_STOP);

	return 0;
}

static void strat_cmd(void* arg)
{
	(void) arg;
	// TODO test des actions unitairement
}
