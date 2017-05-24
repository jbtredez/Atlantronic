#include "disco/action/MoveBackward.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "kernel/systick.h"
#include "kernel/log.h"
#include "kernel/driver/usb.h"
#include "kernel/location/location.h"
#include "kernel/match.h"
#include "middleware/trajectory/Trajectory.h"
#include "disco/gate/robot_state.h"
#include "disco/gate/servos.h"
#include "middleware/stratege_machine/stratege.h"


#include "disco/gate/action/escapeStart.h"
#include "disco/gate/action/rocket_dismantler.h"
#include "disco/gate/action/drop_module.h"
#include "strat/strat_priority.h"
#include "disco/gate/action/module_harvest.h"
#include "disco/gate/action/nul.h"
#include "disco/action/avoidanceTest.h"
#include "disco/gate/action/jetPackSuck.h"
#include "disco/gate/action/jetPackSpread.h"


#define HOMOLOGATION

#define STRAT_STACK_SIZE       500


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

	// Set Servo torque
	Servos::setTorque(true);
	escs_enable(true);

	//création et chargement des actions à faire
	VectPlan firstcheckpoint;

	firstcheckpoint.x = 0;
	firstcheckpoint.y = 0;
	firstcheckpoint.theta = 0;

	// Sortir de la zone en passant la bascule
//	EscapeStart escapeBase(firstcheckpoint, "Escape from Base", &robothomologation);
	// Recalage
	// Premiere action: Vider la premiere fusée
//	firstcheckpoint.x = 150;
//	firstcheckpoint.y = 600;
//	firstcheckpoint.theta = M_PI_2;
//	RocketDismantler topRocket(firstcheckpoint, 0, "Get Top rocket", &robothomologation);

	// Aller à la deuxieme fusée
	firstcheckpoint.theta = M_PI;
	RocketDismantler sideRocket(firstcheckpoint, 8, "Get Side rocket", &robothomologation);

	// Récupere les module isolés
	firstcheckpoint.x = 500;
	firstcheckpoint.y = 600;
	firstcheckpoint.theta = -M_PI_2;
	ModuleHarvest modHarvest1(firstcheckpoint, 0, "Get first module", &robothomologation);

	firstcheckpoint.x = 800;
	firstcheckpoint.y = 0;
	firstcheckpoint.theta = -0.5;
	ModuleHarvest modHarvest2(firstcheckpoint, 0, "Get second modules", &robothomologation);

	firstcheckpoint.x = 800;
	firstcheckpoint.y = -200;
	firstcheckpoint.theta = -3*M_PI_4;
	ModuleHarvest modHarvest3(firstcheckpoint, 0, "Get third modules", &robothomologation, true);

	// Déposer les cylindres à la base
	DropModule dropModuleBase(firstcheckpoint, 0, "Drop the modules to base", &robothomologation);

	firstcheckpoint.x = 800;
	firstcheckpoint.y = -600;
	firstcheckpoint.theta = M_PI+1.2;
	ModuleHarvest modHarvest4(firstcheckpoint, 0, "Get fourth modules", &robothomologation, true);

	//Nul nulForHomologation(firstcheckpoint, 0, "Get isolates modules", &robothomologation);


	firstcheckpoint.x = 1000;
	firstcheckpoint.y = 200;
	firstcheckpoint.theta = -M_PI / 3;
	JetPackSuck suckBalls1(firstcheckpoint, " Suck the first crater", &robothomologation);

	firstcheckpoint.x = 1100;
	firstcheckpoint.y = 200;
	firstcheckpoint.theta = M_PI/4;
	ModuleHarvest modHarvest5(firstcheckpoint, 0, "Get fourth modules", &robothomologation, true);

	firstcheckpoint.x = 1250;
	firstcheckpoint.y = 400;
	firstcheckpoint.theta = -M_PI_2;
	JetPackSpread spreadBalls1(firstcheckpoint, "Release balls", &robothomologation);


	// Test d'évitement pour l'homologation
	AvoidanceTest avoidance(firstcheckpoint, "Avoidance test", &robothomologation);

	StratPriority strat;

#ifndef HOMOLOGATION

	strat.add_action(&suckBalls1, 123);
	strat.add_action(&modHarvest5, 122);
	strat.add_action(&spreadBalls1, 121);

	strat.add_action(&avoidance, 120);

#else
	strat.add_action(&modHarvest1, 128);
	strat.add_action(&modHarvest2, 127);
	strat.add_action(&modHarvest3, 126);
	strat.add_action(&dropModuleBase, 125);
	strat.add_action(&modHarvest4, 124);
	strat.add_action(&suckBalls1, 123);
	strat.add_action(&modHarvest5, 122);
	strat.add_action(&spreadBalls1, 121);
//	strat.add_action(&avoidance, 126);

#endif

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

