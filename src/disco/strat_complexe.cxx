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


#include "strat/strat_simple.h"


#define STRAT_STACK_SIZE       300

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
	//Clapet 1
	VectPlan firstcheckpoint(730 ,-785,0.0f);
	clapet clap1(firstcheckpoint,&robothomologation);


	//Dropzone startzone 
	firstcheckpoint.x = 950;
	firstcheckpoint.y = 0;
	dropzone dropstartzone(firstcheckpoint,&robothomologation);



	//light 1
	firstcheckpoint.x = 1285 + LIGHT_APPROX_DIST;
	firstcheckpoint.y = 0;
	light light1(firstcheckpoint,&robothomologation);

	//Pied 1
	firstcheckpoint.x = 630;
	firstcheckpoint.y = -355;
	feet feet1(firstcheckpoint,&robothomologation);

	//Pied 2 
	firstcheckpoint.x = 200;
	firstcheckpoint.y = -400;
	feet feet2(firstcheckpoint,&robothomologation);

	//Pied 3 
	firstcheckpoint.x = 400;
	firstcheckpoint.y = -750;
	feet feet3(firstcheckpoint,&robothomologation);

	//Pied 4 
	firstcheckpoint.x = 1410;
	firstcheckpoint.y = -750;
	feet feet4(firstcheckpoint,&robothomologation);

	//Pied 5 
	firstcheckpoint.x = 1410;
	firstcheckpoint.y = -850;
	feet feet4(firstcheckpoint,&robothomologation);

	//start
	firstcheckpoint.x = 1000;
	firstcheckpoint.y = 0;
	movebackward startzone(firstcheckpoint);

	//Création du spotlight
	spotlight spotlight1(firstcheckpoint,&robothomologation);

	spotlight1.add_action(&light1);	
	spotlight1.add_action(&feet1);	
	spotlight1.add_action(&feet2);	
	spotlight1.add_action(&feet3);	
	spotlight1.add_action(&feet4);
	spotlight1.add_action(&dropposition);



	//light 2
	firstcheckpoint.x = 250 + LIGHT_APPROX_DIST;
	firstcheckpoint.y = -800;
	light light2(firstcheckpoint,&robothomologation);


	//Pied 6
	firstcheckpoint.x = 650;
	firstcheckpoint.y = 800;
	feet feet6(firstcheckpoint,&robothomologation);

	//Pied 7 
	firstcheckpoint.x = 650;
	firstcheckpoint.y = 900;
	feet feet7(firstcheckpoint,&robothomologation);

	//Pied 8 
	firstcheckpoint.x = 1410;
	firstcheckpoint.y = 800;
	feet feet8(firstcheckpoint,&robothomologation);
	

	//Actoin spolight2 
	spotlight spotlight1(firstcheckpoint,&robothomologation);
	spotlight2.add_action(&light2);	
	spotlight2.add_action(&feet6);	
	spotlight2.add_action(&feet7);	
	spotlight2.add_action(&feet8);
	spotlight2.add_action(&dropstartzone);


	//Action gobelet 1
	firstcheckpoint.x = 590;
	firstcheckpoint.y = 200;
	gobelet gobelet1(firstcheckpoint,&robothomologation);

	feed feedgob1(firstcheckpoint,&robothomologation);
	feedgob1.add_action(&gobelet1);	
	feedgob1.add_action(&dropstartzone);	

	//Action gobelet 2

	//Dropzone startzone 
	firstcheckpoint.x = 950;
	firstcheckpoint.y = 0;
	dropzone dropcinemazone(firstcheckpoint,&robothomologation);


	firstcheckpoint.x = 0;
	firstcheckpoint.y = -650;
	gobelet gobelet2(firstcheckpoint,&robothomologation);

	feed feedgob2(firstcheckpoint,&robothomologation);
	feedgob2.add_action(&gobelet1);	
	feedgob2.add_action(&dropcinemazone);	

	//Action gobelet 3

	firstcheckpoint.x = 950;
	firstcheckpoint.y = 0;
	dropzone dropcinemazone2(firstcheckpoint,&robothomologation);

	firstcheckpoint.x = 1250;
	firstcheckpoint.y = -750;
	gobelet gobelet2(firstcheckpoint,&robothomologation);

	feed feedgob2(firstcheckpoint,&robothomologation);
	feedgob2.add_action(&gobelet1);	
	feedgob2.add_action(&dropcinemazone2);	
	

	//Action carpette
	firstcheckpoint.x = 250;
	firstcheckpoint.y = 200;	
	deposecarpette carpette1(firstcheckpoint,&robothomologation);
	

	stratcomplexe_def strat;


	strat.add_action(&spotlight1);
	strat.add_action(&startzone);
	strat.add_action(&clap1);
	strat.add_action(&carpette1);
	strat.add_action(&spotlight2);
	strat.add_action(&feedgob1);





	match_wait_go();
	strat_color = match_get_color();
	log_format(LOG_ERROR, "couleur %d", (int)strat_color );
	
	

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
