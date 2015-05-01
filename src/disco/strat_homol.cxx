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
#include "clapet.h"
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
static int strat_clap1(void* arg);
static int strat_demarage(void* arg);

static int strat_color;
StratAction strat_action[ ] =
{

    { "demarage", strat_demarage, NULL, -1},
    //		{ "start", strat_start, NULL, -1},
        { "clap", strat_clap1, NULL, -1},
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

	VectPlan firstcheckpoint(730 * strat_color,-785,0.0f);
	clapet clap1(firstcheckpoint);
	firstcheckpoint.x = -1030 * strat_color;
	clapet clap2(firstcheckpoint);
	// realisation des actions dans l'ordre au debut
	for(unsigned int i = 0; i < sizeof(strat_action)/sizeof(strat_action[0]); i++)
	{
		StratAction* a = &strat_action[i];
		log_format(LOG_INFO, "action %s", a->name);
		a->action(a->arg);
	}

	clap2.do_action();
	clap1.do_action();
	// TODO faire les actions manquantes jusqu'a la fin du match
	while(1)
	{
		vTaskDelay(100);
	}
}
static int strat_demarage(void* /*arg*/)
{
    // sortie case depart en marche arriere
    VectPlan dest(1000, 0, 0);
    trajectory_goto(dest.symetric(strat_color), WAY_BACKWARD, AVOIDANCE_STOP);
    trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000); // TODO verif cas erreur
    return 0;
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

	// depose case depart
	trajectory_goto_near_xy(strat_color * 1000, 0.0f, 0.0f, WAY_FORWARD, AVOIDANCE_STOP);
	trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000); // TODO verif cas erreur
	finger_set_pos(FINGER_OPEN, FINGER_OPEN);
	vTaskDelay(500);
	trajectory_straight(-100);



	return 0;
}

static bool strat_Oneclap(int axpos)
{
	bool bresult = true;
	int essaie =0;
	float angle = 0.0f;
	int second_x_position = 0;
   	//On se déplace sur le vecteur y vers l'origine de la taille du clap -160 *stratcolor (vert vers y négatifs,jaune vers les y positifs) ou un décalage postif si le y est négatif
	if(axpos > 0)
	{
		second_x_position = axpos - 185;
		angle = -3.14f;
	}
	else
	{
		second_x_position = axpos + 185;
		
	}
        VectPlan nextToClap(axpos, -785, angle );
	//Mise en place de la position
        trajectory_goto(nextToClap, WAY_FORWARD, AVOIDANCE_STOP);

   	//On ouvre nos ailes, pas besoin de réflechir de quel coté on est(homologation).
 
	//Si on arrive pas à joindre le clapet on abandonne
	if(trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 20000) != 0)
	{
		log_format(LOG_ERROR, "on impossible de rejoindre la position %d", axpos);
		return false; 
	 }
	//On ouvre l'aile pas besoin de réfléchir
   	wing_set_position(WING_OPEN, WING_OPEN);


	nextToClap.x = second_x_position;

	//On essaie de se déplacer 3 fois afin d'abandonner
	do
	{
    		trajectory_goto(nextToClap, WAY_FORWARD, AVOIDANCE_STOP);
		if(trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) == 0)
		{
			bresult = true;
		}
		else
		{
			log_format(LOG_ERROR, "on impossible de rejoindre la position %d", (int)nextToClap.x);
			bresult = false;
		}
	
		essaie++;
	}while(essaie <= 3 && !bresult); 

	//On ferme l'aile pas besoin de réfléchir
	wing_set_position(WING_PARK, WING_PARK);

	return bresult;
}



static int strat_clap1(void* /*arg*/)
{	

	log_format(LOG_INFO, "color %d", strat_color);
    //VectPlan nextToClap(730 * strat_color, -770, 0);
    //Mise en place de la position
    //trajectory_goto(nextToClap, WAY_FORWARD, AVOIDANCE_STOP);

    //trajectory_goto_graph_node(13, 0, WAY_BACKWARD, AVOIDANCE_STOP);
   // trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000); // TODO verif cas erreur

  //  strat_Oneclap(1300 * strat_color);



    strat_Oneclap(730 * strat_color);
 

    strat_Oneclap(-1030 * strat_color);


	return 0;
}

static void strat_cmd(void* /*arg*/)
{
	// TODO test des actions unitairement
}
