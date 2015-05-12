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
	clapet clap1(firstcheckpoint,&robothomologation);




	//start
	firstcheckpoint.x = 1000;
	firstcheckpoint.y = 0;
	movebackward startzone(firstcheckpoint);


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
	firstcheckpoint.y = -770;
	feet feet3(firstcheckpoint,&robothomologation);
	
	//Dropstart 
	firstcheckpoint.x = 950;
	firstcheckpoint.y = 0;
	dropzone dropstartzone(firstcheckpoint,&robothomologation);

	

	stratsimple strat;


	strat.add_action(&light1);
	strat.add_action(&startzone);
	strat.add_action(&feet1);
	strat.add_action(&feet2);
	strat.add_action(&feet3);
	strat.add_action(&clap1);
	strat.add_action(&dropstartzone);



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
