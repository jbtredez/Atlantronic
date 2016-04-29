#include "disco/star/action/Light.h"
#include "disco/star/action/MoveBackward.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "kernel/systick.h"
#include "kernel/log.h"
#include "kernel/driver/usb.h"
#include "kernel/location/location.h"
#include "kernel/match.h"
#include "middleware/trajectory/Trajectory.h"
#include "disco/star/robot_state.h"
#include "middleware/stratege_machine/stratege.h"


#include "disco/star/action/Move.h"
#include "disco/star/action/hut.h"
#include "disco/star/action/fishes.h"
#include "disco/star/action/fellowCastle.h"
#include "disco/star/action/dropCastle.h"

#include "strat/strat_simple.h"


#define STRAT_STACK_SIZE       500

//Action : recherche la balle, les pieds puis basse le clapet et fini par déposer le spotlight dans la zone principale



static void strat_task(void* arg);
static void strat_cmd(void* arg, void* data);

static int strat_color;

int strat_module_init()
{
	portBASE_TYPE err = xTaskCreate(strat_task, "strat0", STRAT_STACK_SIZE, NULL, PRIORITY_TASK_STRATEGY, NULL);

	if(err != pdPASS)
	{
		return ERR_INIT_STRAT;
	}

	strat_color = match_get_color();

	usb_add_cmd(USB_CMD_STRAT, strat_cmd, NULL);

	return 0;
}

module_init(strat_module_init, INIT_STRATEGY);

static void strat_task(void* arg)
{
	(void) arg;

	RobotState robothomologation;

	//création et chargement des actions à faire
	VectPlan firstcheckpoint(-900,600,-M_PI/2);

	// Cabanes
	Hut hut1(firstcheckpoint, "Pull huts", &robothomologation);

	// Poissons
	Fishes fishes(firstcheckpoint, "Fishes", &robothomologation);

	//Chateau ami
	FellowCastle fellowCastle(firstcheckpoint, "Fellow Castle", &robothomologation);

	firstcheckpoint.x = 400;
	firstcheckpoint.y = -200;
	firstcheckpoint.theta = -M_PI_4;
	// Depose chateau 1
	DropCastle dropCastle1(firstcheckpoint, "Drop first checkpoint", &robothomologation);

	StratSimple strat;

	//strat.add_action(&hut1);
	//strat.add_action(&fishes);
	strat.add_action(&fellowCastle);
	strat.add_action(&dropCastle1);

	match_wait_go();
	strat_color = match_get_color();
	log_format(LOG_INFO, "couleur %d", (int)strat_color );
	

	strat.Initialise(strat_color);
	strat.affiche();
	strat.run();


	while(1)
	{
//		wing_set_position(WING_OPEN, WING_OPEN);
		log_format(LOG_INFO, "WAIT NEW ORDER");
		vTaskDelay(1000);
//		wing_set_position(WING_PARK, WING_PARK);
		vTaskDelay(1000);
	}
}




static void strat_cmd(void* /*arg*/, void* /*data*/)
{
	// TODO test des actions unitairement
}

