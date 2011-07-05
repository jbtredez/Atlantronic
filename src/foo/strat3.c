#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "control/control.h"
#include "kernel/systick.h"
#include "kernel/event.h"
#include "gpio.h"
#include "location/location.h"
#include "kernel/rcc.h"
#include <math.h>
#include "kernel/robot_parameters.h"
#include "pince.h"
#include "control/control_pince.h"
#include "recalage.h"
#include "kernel/hokuyo_tools.h"
#include "us.h"

#define STRAT_STACK_SIZE       300

static void strat_task();
int strat_module_init();
int pos_tour;

int strat_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(strat_task, "homol", STRAT_STACK_SIZE, NULL, PRIORITY_TASK_STRATEGY, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_TEST;
	}

	return 0;
}

module_init(strat_module_init, INIT_STRATEGY);
float get_distance();

void goto_with_avoidance(float x, float y, float delta, int dir)
{
	uint32_t event;
	portTickType start = systick_get_time();
	portTickType tempo_colision;
	portTickType stop;

	do
	{
		control_goto_near(x, y, delta, dir);
		event = vTaskWaitEvent(EVENT_CONTROL_READY, ms_to_tick(8000));
		stop = systick_get_time();
		tempo_colision = stop - start;
	}while( (event & EVENT_CONTROL_COLSISION) && (event & EVENT_CONTROL_READY) && tempo_colision > ms_to_tick(1000));

	// tentative de debloquage
	if( (! (event & EVENT_CONTROL_READY)) || (event & EVENT_CONTROL_COLSISION))
	{
		control_free();
		vTaskDelay(ms_to_tick(500));
		// bloquage
		control_straight(-300);
		vTaskWaitEvent(EVENT_CONTROL_READY, ms_to_tick(2000));
	}
	if( (! (event & EVENT_CONTROL_READY)) || (event & EVENT_CONTROL_COLSISION))
	{
		// bloquage
		control_free();
		vTaskDelay(ms_to_tick(500));
		control_rotate(PI/2.0f);
		control_straight(300);
		vTaskWaitEvent(EVENT_CONTROL_READY, ms_to_tick(3000));
	}
	if( (! (event & EVENT_CONTROL_READY)) || (event & EVENT_CONTROL_COLSISION) )
	{
		// bloquage
		control_free();
		vTaskDelay(ms_to_tick(500));
		control_rotate(PI);
		control_straight(300);
		vTaskWaitEvent(EVENT_CONTROL_READY, ms_to_tick(3000));
	}
	if( (! (event & EVENT_CONTROL_READY)) || (event & EVENT_CONTROL_COLSISION))
	{
		control_free();
		vTaskDelay(ms_to_tick(500));
		control_goto_near(x, y, delta, dir);
		vTaskWaitEvent(EVENT_CONTROL_READY, ms_to_tick(8000));
	}
}

void straight_with_avoidance(float dx)
{
	uint32_t event;
	do
	{
		control_straight(dx);
		event = vTaskWaitEvent(EVENT_CONTROL_READY, ms_to_tick(8000));
	}while(event & EVENT_CONTROL_COLSISION && event & EVENT_CONTROL_READY);
}


void action_first_pawn(int sens)
{
	int first_pawn = 0;

	// on avance sur la sortie de la case depart
	goto_with_avoidance(- sens * 1100, -850, 0, CONTROL_FORWARD);

	// si on a detecté un pion au lancement, on va le prendre
	if( is_pawn_front_start() )
	{
		goto_with_avoidance(- sens * 700, -700, 140, CONTROL_FORWARD);	
		pince_close();
		vTaskDelay(ms_to_tick(150));
		if( pince_full() )
		{
			first_pawn = 1;
			goto_with_avoidance(- sens * 575, -875, 160, CONTROL_FORWARD);
			pince_open();
			straight_with_avoidance(-200);
			goto_with_avoidance(- sens * 700, -350, 0, CONTROL_FORWARD);
		}
	}

	if( first_pawn == 0)
	{
		goto_with_avoidance(- sens * 700, -700, 0, CONTROL_FORWARD);
		control_rotate_to(PI/2);
		vTaskWaitEvent(EVENT_CONTROL_READY,  ms_to_tick(4000));

		//Attente scan Hokuyo
		vTaskDelay(ms_to_tick(400));
		float dist = get_distance();
		float pos_pawn = 0.0f;
		if( dist > 0)
		{
			dist += 60;
			pos_pawn = -700 + dist + 100;
			if( fabsf(pos_pawn + 350) < fabsf(pos_pawn) )
			{
				goto_with_avoidance(- sens * 700, -350, 160, CONTROL_FORWARD);
				pince_close();
				goto_with_avoidance(- sens * 175, 130, 160, CONTROL_FORWARD);
				pince_open();
				goto_with_avoidance(- sens * 700, -350, 0, CONTROL_BACKWARD);
			}
			else if (fabsf(pos_pawn) < fabsf(pos_pawn - 350))
			{
				goto_with_avoidance(- sens * 700, 0, 160, CONTROL_FORWARD);
				pince_close();
				goto_with_avoidance(- sens * 175, 130, 160, CONTROL_FORWARD);
				pince_open();
				goto_with_avoidance(- sens * 700, 0, 0, CONTROL_BACKWARD);
			}
			else
			{
				goto_with_avoidance(- sens * 700, 350, 160, CONTROL_FORWARD);
				pince_close();
				goto_with_avoidance(- sens * 175, 130, 160, CONTROL_FORWARD);
				pince_open();
				goto_with_avoidance(- sens * 700, 0, 0, CONTROL_BACKWARD);
			}
		}
		else
		{
			goto_with_avoidance(- sens * 700, 350, 160, CONTROL_FORWARD);
			pince_close();
			goto_with_avoidance(- sens * 175, 130, 160, CONTROL_FORWARD);
			pince_open();
			goto_with_avoidance(- sens * 700, 0, 0, CONTROL_BACKWARD);
		}
	}

	goto_with_avoidance(- sens * 700, 700, 160, CONTROL_FORWARD);

	pince_close();
	straight_with_avoidance(-400);

	// placement sur le point noir cote bordure
	goto_with_avoidance(- sens * 130, 475, 0, CONTROL_FORWARD);
	goto_with_avoidance(- sens * 130, 875, 160, CONTROL_FORWARD);

	pince_open();
	// prise pion ds zone verte, case 3
	goto_with_avoidance(- sens * 150, 400, 160, CONTROL_BACKWARD);
	control_set_use_us(US_BACK_MASK);
	goto_with_avoidance(- sens * 1380, 480, 160, CONTROL_FORWARD);
	pince_close();
	vTaskDelay(ms_to_tick(300));
	straight_with_avoidance(-100);
	goto_with_avoidance(- sens * 875, 480, 0, CONTROL_BACKWARD);
	control_set_use_us(US_FRONT_MASK | US_BACK_MASK);
	goto_with_avoidance(- sens * 875, -525, 110, CONTROL_FORWARD);
	pince_open();

	// prise pion ds zone verte, case 0
	straight_with_avoidance(-300);
	control_set_use_us(US_BACK_MASK);
	goto_with_avoidance(- sens * 1380, -350, 160, CONTROL_FORWARD);
	pince_close();
	vTaskDelay(ms_to_tick(300));
	control_set_use_us(US_FRONT_MASK | US_BACK_MASK);
	straight_with_avoidance(-300);
	goto_with_avoidance(- sens * 525, -175, 160, CONTROL_FORWARD);
	pince_open();
	straight_with_avoidance(-300);

	// prise pion ds zone verte, case 1
	goto_with_avoidance(- sens * 875, -80, 0, CONTROL_FORWARD);
	control_set_use_us(US_BACK_MASK);
	goto_with_avoidance(- sens * 1380, -80, 160, CONTROL_FORWARD);
	pince_close();
	vTaskDelay(ms_to_tick(300));
	straight_with_avoidance(-100);
	control_set_use_us(US_FRONT_MASK | US_BACK_MASK);
	goto_with_avoidance(- sens * 875, 175, 0, CONTROL_BACKWARD);

	goto_with_avoidance(- sens * 680, 175, 0, CONTROL_BACKWARD);

	goto_with_avoidance(- sens * 875, 175, 160, CONTROL_FORWARD);
	pince_open();
	
	// on va de l'autre cote
	goto_with_avoidance(- sens * 525, 175, 0, CONTROL_BACKWARD);
	goto_with_avoidance( sens * 525, 175, 160, CONTROL_FORWARD);
	goto_with_avoidance( sens * 125, 175, 0, CONTROL_BACKWARD);

	// on va piquer un pion sur le point noir coté bordure
	control_set_use_us(US_BACK_MASK);
	goto_with_avoidance( sens * 175, 525, 0, CONTROL_FORWARD);
	pince_close();
	vTaskDelay(ms_to_tick(300));
	if( pince_full() == 0 )
	{
		pince_open();
		vTaskDelay(ms_to_tick(200));
		goto_with_avoidance( sens * 175, 875, 60, CONTROL_FORWARD);
		pince_close();
		straight_with_avoidance(-100);
		goto_with_avoidance( sens * 175, 525, -160, CONTROL_BACKWARD);
	}
	else
	{
		straight_with_avoidance(-150);
	}
	pince_open();
	control_set_use_us(US_FRONT_MASK | US_BACK_MASK);
	goto_with_avoidance( sens * 175, 175, 0, CONTROL_BACKWARD);
	goto_with_avoidance( sens * 175, -875, 160, CONTROL_FORWARD);
}

static void strat_task()
{
	int sens = 1;

	while(getGo() == 0)
	{
		if( getRecalage() )
		{
			recalage();
			resetRecalage();
		}
		vTaskDelay(ms_to_tick(50));
	}

	vTaskWaitEvent(EVENT_GO, portMAX_DELAY);

	if(getcolor() != COLOR_BLUE)
	{
		sens = -1;
	}

	pince_open();
	action_first_pawn(sens);

	pince_open();

	vTaskDelete(NULL);
}

