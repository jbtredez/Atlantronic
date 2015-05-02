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
#include "kernel/stratege_machine/stratege.h"
#include "action/clapet.h"
#include "strat_simple.h"
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



static int strat_color;

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

	//création et chargement des actions à faire
	VectPlan firstcheckpoint(730 ,-785,0.0f);	
	clapet clap1(firstcheckpoint);
	firstcheckpoint.x = -1030 ;
	clapet clap2(firstcheckpoint);

	homologation strat;
	strat.add_action(&clap1);
	strat.add_action(&clap2);



	match_wait_go();
	strat_color = match_get_color();
	log_format(LOG_ERROR, "couleur %d", (int)strat_color );
	
	

	strat.Initialise(strat_color);
	strat.run();
	



	while(1)
	{
		wing_set_position(WING_OPEN, WING_OPEN);
		log_format(LOG_ERROR, "WAIT NEW ORDER");
		vTaskDelay(1000);
		wing_set_position(WING_PARK, WING_PARK);
		vTaskDelay(1000);
	}
}




static void strat_cmd(void* /*arg*/)
{
	// TODO test des actions unitairement
}
