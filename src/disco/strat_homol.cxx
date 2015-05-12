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

#include "disco/robot_state.h"
#include "kernel/stratege_machine/stratege.h"

#include "disco/action/clapet.h"
#include "disco/action/feet.h"
#include "disco/action/dropzone.h"
#include "disco/action/movebackward.h"
#include "disco/action/light.h"
#include "disco/action/deposecarpette.h"


#include "strat/strat_simple.h"


#define STRAT_STACK_SIZE       500

//Action : recherche la balle, les pieds puis basse le clapet et fini par déposer le spotlight dans la zone principale



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

	robotstate robothomologation;
	robothomologation.setnumberelement(0);
	robothomologation.setelevatorstate(ELEVATOR_LIGHT);


	//création et chargement des actions à faire
	VectPlan firstcheckpoint(730 ,-770,0.0f);
	char Cclapet1[] = "Clapet 1";
	clapet clap1(firstcheckpoint,Cclapet1,&robothomologation);

	//start
	firstcheckpoint.x = 1000;
	firstcheckpoint.y = 0;
	char Cstarzone[] = "Startzone";
	movebackward startzone(firstcheckpoint,Cstarzone);


	//light 1
	firstcheckpoint.x = 1285 + LIGHT_APPROX_DIST;
	firstcheckpoint.y = 0;
	firstcheckpoint.theta = 0
	char CLight1[] = "Light 1";
	light light1(firstcheckpoint,CLight1,&robothomologation);


	//Pied 1
	firstcheckpoint.x = 630;
	firstcheckpoint.y = -355;
	char CFeet1[] = "Feet 1";
	feet feet1(firstcheckpoint,CFeet1,&robothomologation);

	//Pied 2 
	firstcheckpoint.x = 200;
	firstcheckpoint.y = -400;
	char CFeet2[] = "Feet 2";
	feet feet2(firstcheckpoint,CFeet2,&robothomologation);

	//Pied 3 
	firstcheckpoint.x = 400;
	firstcheckpoint.y = -770;
	char CFeet3[] = "Feet 3";
	feet feet3(firstcheckpoint,CFeet3,&robothomologation);
	
	//Dropstart 
	firstcheckpoint.x = 950;
	firstcheckpoint.y = 0;
	char CDropstart[] = "Feet 3";
	dropzone dropstartzone(firstcheckpoint,CDropstart,&robothomologation);

 // TODO tapis : OK mais on passe sur un verre avant d'y aller...
	// Carpet1
	firstcheckpoint.x = 390;
	firstcheckpoint.y = 200;
	char CCarpetLeft[] = "Capette Left";
	deposecarpette carpet1(firstcheckpoint,CCarpetLeft,&robothomologation, false);

	// Carpet2
	firstcheckpoint.x = 110;
	firstcheckpoint.y = 200;
	char CCarpetRight[] = "Capette Right";
	deposecarpette carpet2(firstcheckpoint,CCarpetRight,&robothomologation, true);

	stratsimple strat;

	strat.add_action(&light1);
	strat.add_action(&startzone);
	strat.add_action(&feet1);
	strat.add_action(&feet2);
	strat.add_action(&feet3);
	strat.add_action(&clap1);
	strat.add_action(&dropstartzone);
	strat.add_action(&carpet1);
	strat.add_action(&carpet2);

	match_wait_go();
	strat_color = match_get_color();
	log_format(LOG_INFO, "couleur %d", (int)strat_color );
	

	strat.Initialise(strat_color);
	strat.run();


	while(1)
	{
		wing_set_position(WING_OPEN, WING_OPEN);
		log_format(LOG_INFO, "WAIT NEW ORDER");
		vTaskDelay(1000);
		wing_set_position(WING_PARK, WING_PARK);
		vTaskDelay(1000);
	}
}




static void strat_cmd(void* /*arg*/)
{
	// TODO test des actions unitairement
}
