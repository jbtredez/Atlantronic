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
#include "disco/action/gobelet.h"

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
	robothomologation.setelevatorstate(ELEVATOR_EMPTY);


	//création et chargement des actions à faire
	VectPlan firstcheckpoint(690 ,-780,0.0f);
	Clapet clap1(firstcheckpoint, "Clapet 1", &robothomologation);

	//start
	firstcheckpoint.x = 1000;
	firstcheckpoint.y = 0;
	movebackward startzone(firstcheckpoint, "Startzone");

	//light 1
	firstcheckpoint.x = 1285;
	firstcheckpoint.y = 0;
	firstcheckpoint.theta = 0;
	light light1(firstcheckpoint, "Light 1", &robothomologation, false);


	//Pied 1
	firstcheckpoint.x = 630;
	firstcheckpoint.y = -355;
	feet feet1(firstcheckpoint, "Feet 1", &robothomologation);

	//Pied 2 
	firstcheckpoint.x = 200;
	firstcheckpoint.y = -400;
	feet feet2(firstcheckpoint, "Feet 2", &robothomologation);

	//Pied 3 
	firstcheckpoint.x = 400;
	firstcheckpoint.y = -770;
	feet feet3(firstcheckpoint, "Feet 3", &robothomologation);

	//Dropstart 
	firstcheckpoint.x = 850;
	firstcheckpoint.y = 0;
	DropZone dropstartzone(firstcheckpoint, "drop start", &robothomologation);


	//Dropcinema 
	firstcheckpoint.x = -1000;
	firstcheckpoint.y = -400;
	DropZone dropcinema(firstcheckpoint, "drop cinema", &robothomologation);


	// Goblet
	firstcheckpoint.x = 590;
	firstcheckpoint.y = 170;
	Gobelet gobelet1(firstcheckpoint, "gobelet 1", &robothomologation);

	// Carpet1
	firstcheckpoint.x = 390;
	firstcheckpoint.y = 200;
	DeposeCarpette carpet1(firstcheckpoint, "Capette Left", &robothomologation, false);

	// Carpet2
	firstcheckpoint.x = 110;
	firstcheckpoint.y = 200;
	DeposeCarpette carpet2(firstcheckpoint, "Capette Right", &robothomologation, true);

	// Goblet 2
	firstcheckpoint.x = 1250;
	firstcheckpoint.y = -750;
	Gobelet gobelet2(firstcheckpoint, "gobelet 2", &robothomologation);

	firstcheckpoint.x = 1270;
	firstcheckpoint.y = -780;
	Clapet clap2(firstcheckpoint, "Clapet 2", &robothomologation);

	// Goblet 3
	firstcheckpoint.x = 0;
	firstcheckpoint.y = -650;
	Gobelet gobelet3(firstcheckpoint, "gobelet 3", &robothomologation);

	firstcheckpoint.x = -990;
	firstcheckpoint.y = -780;
	Clapet clap3(firstcheckpoint, "Clapet 3", &robothomologation);

/*
	//light 2
	firstcheckpoint.x = 250;
	firstcheckpoint.y = -560;
	firstcheckpoint.theta = -M_PI_2;
	light light2(firstcheckpoint, "Light 2", &robothomologation, true);

	//Pied 4
	firstcheckpoint.x = 710;
	firstcheckpoint.y = 800;
	feet feet4(firstcheckpoint, "Feet 4", &robothomologation);
*/
	stratsimple strat;

	strat.add_action(&light1);
	strat.add_action(&startzone);
	strat.add_action(&feet1);
	strat.add_action(&feet2);
	strat.add_action(&feet3);
	strat.add_action(&clap1);
	strat.add_action(&dropstartzone);
	strat.add_action(&gobelet1);
	strat.add_action(&carpet1);
	strat.add_action(&carpet2);
	strat.add_action(&dropstartzone);
//	strat.add_action(&gobelet2);
//	strat.add_action(&clap2);
//	strat.add_action(&dropstartzone);
	strat.add_action(&gobelet3);
	strat.add_action(&clap3);
	strat.add_action(&dropcinema);
//	strat.add_action(&light2);
//	strat.add_action(&feet4);

	match_wait_go();
	strat_color = match_get_color();
	log_format(LOG_INFO, "couleur %d", (int)strat_color );
	

	strat.Initialise(strat_color);
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




static void strat_cmd(void* /*arg*/)
{
	// TODO test des actions unitairement
}
