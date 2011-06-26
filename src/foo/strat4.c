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
#include "detection.h"

#define STRAT_STACK_SIZE       300

static void strat_task();
int strat_module_init();
int pos_tour;

int strat_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(strat_task, "strat4", STRAT_STACK_SIZE, NULL, PRIORITY_TASK_STRATEGY, &xHandle);

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

int take_pawn()
{
	int res = 0;

	pince_close();
	vTaskDelay(ms_to_tick(150));
	if( pince_full() )
	{
		res = 1;
	}
	else
	{
		pince_open();
	}

	return res;
}

int pos_tour;

void action_first_pawn(int sens)
{
	int first_pawn = 0;

	// on avance sur la sortie de la case depart
	goto_with_avoidance(- sens * 1100, -850, 0, CONTROL_FORWARD);

	// si on a detecté un pion au lancement, on va le prendre
	if( is_pawn_front_start() )
	{
		goto_with_avoidance(- sens * 700, -700, 140, CONTROL_FORWARD);	

		first_pawn = take_pawn();

		if( first_pawn )
		{
			control_pince_dual(PINCE_POS_HI, 0);
		}
	}

	goto_with_avoidance(- sens * 700, -700, 0, CONTROL_FORWARD);
	control_rotate_to(PI/2);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	if(getcolor() == COLOR_BLUE)
	{
		us_start_scan(US_LEFT_MASK);
	}
	else
	{
		us_start_scan(US_RIGHT_MASK);
	}

	if(  ! first_pawn )
	{
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
			}
			else if (fabsf(pos_pawn) < fabsf(pos_pawn - 350))
			{
				goto_with_avoidance(- sens * 700, 0, 160, CONTROL_FORWARD);
			}
			else
			{
				goto_with_avoidance(- sens * 700, 350, 160, CONTROL_FORWARD);
			}
		}
		else
		{
			goto_with_avoidance(- sens * 700, 350, 160, CONTROL_FORWARD);
		}
		pince_close();
		control_pince_dual(PINCE_POS_HI, 0);
		vTaskDelay(ms_to_tick(300));
	}

	// tout droit vers la derniere instersection ligne 1
	goto_with_avoidance(- sens * 700, 700, 160, CONTROL_FORWARD);

	// on arrete le scan des pions
	pos_tour = us_get_scan_result();

	// placement du 2ème palet en zone securisé
	goto_with_avoidance(- sens * 875, 830, 160, CONTROL_FORWARD);

	// on recule
	straight_with_avoidance(-300);
}

void action_build_tower(int sens)
{
	int pawn_pos_1 = 0;
	
	goto_with_avoidance(- sens * 525, 525, 0, CONTROL_BACKWARD);

	control_pince_dual(PINCE_POS_LOW, 0);
	control_rotate_to(PI/2);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);

	// on recule
	straight_with_avoidance(-160);
	pince_open();
	straight_with_avoidance(-300);
	
	switch(pos_tour)
	{
		case 0:
			goto_with_avoidance(- sens * 700, -350, 0, CONTROL_ANY_WAY);
			goto_with_avoidance(- sens * 1400, -350, 160, CONTROL_FORWARD);
			break;
		case 1:
			pawn_pos_1 = 1;
			goto_with_avoidance(- sens * 700, -80, 0, CONTROL_ANY_WAY);
			goto_with_avoidance(- sens * 1400, -80, 0, CONTROL_FORWARD);
			break;
		case 2:
			goto_with_avoidance(- sens * 875, 200, 0, CONTROL_ANY_WAY);
			goto_with_avoidance(- sens * 1400, 200, 0, CONTROL_FORWARD);
			break;
		case 3:
			goto_with_avoidance(- sens * 875, 200, 0, CONTROL_FORWARD);
			goto_with_avoidance(- sens * 875, 480, 0, CONTROL_FORWARD);
			goto_with_avoidance(- sens * 1400, 480, 0, CONTROL_FORWARD);
			break;
	}

	take_pawn();
	control_pince_dual(PINCE_POS_HI, 0);
	straight_with_avoidance(-415);

	vTaskDelay(ms_to_tick(400));
	int fig = 0;
	if(us_get_state(US_FRONT) < 200 )
	{
		fig = 1;
	}

	if( fig )
	{
		control_set_use_us(US_BACK_MASK);
		goto_with_avoidance(- sens * 525, 525, 160, CONTROL_FORWARD);
		pince_open();
		control_pince_dual(PINCE_POS_LOW, 0);
		vTaskDelay(ms_to_tick(400));
		pince_close();

		// placement sur le point noir cote bordure
		goto_with_avoidance(- sens * 100, 475, 0, CONTROL_FORWARD);
		goto_with_avoidance(- sens * 250, 950, 160, CONTROL_FORWARD);
		pince_open();
		control_set_use_us(US_BACK_MASK | US_FRONT_MASK);
	}
	else
	{
	    goto_with_avoidance(- sens * 875, -525, 110, CONTROL_FORWARD);
		pince_open();	

		if ( pawn_pos_1 )
		{
			//aller en pos2
			goto_with_avoidance(- sens * 875, 200, 0, CONTROL_BACKWARD);
			goto_with_avoidance(- sens * 1400, 200, 0, CONTROL_FORWARD);
		}
		else
		{
			//aller en pos1
			goto_with_avoidance(- sens * 875, -80, 0, CONTROL_BACKWARD);
			goto_with_avoidance(- sens * 1400, -80, 0, CONTROL_FORWARD);
		}
		
		take_pawn();
		control_pince_dual(PINCE_POS_HI, 0);
		straight_with_avoidance(-415);

		vTaskDelay(ms_to_tick(400));
		fig = 0;
		if(us_get_state(US_FRONT) < 200 )
		{
			fig = 1;
		}
		
		if ( fig )
		{
			control_set_use_us(US_BACK_MASK);
			// on retourne sur noter premier pion	
			goto_with_avoidance(- sens * 525, 525, 160, CONTROL_FORWARD);
			pince_open();
			control_pince_dual(PINCE_POS_LOW, 0);
			vTaskDelay(ms_to_tick(400));
			pince_close();
		}
		else
		{
			goto_with_avoidance(- sens * 175, 175, 160, CONTROL_FORWARD);
		}
		
		// placement sur le point noir cote bordure
		goto_with_avoidance(- sens * 100, 475, 0, CONTROL_FORWARD);
		goto_with_avoidance(- sens * 250, 950, 160, CONTROL_FORWARD);
		pince_open();
		control_set_use_us(US_BACK_MASK | US_FRONT_MASK);
	}
	
	straight_with_avoidance(-400);
}

void cambriolage_adversaire(int sens)
{
	// vol dans le point noir à côté de nous

//	goto_with_avoidance(sens * 100, 475, 0, CONTROL_ANY_WAY); // a voir si besoin de recalage
	control_set_use_us(US_BACK_MASK);
	goto_with_avoidance(sens * 250, 950, 160, CONTROL_FORWARD);
	
	if ( take_pawn() )
	{
		control_pince_dual(PINCE_POS_HI, 0);
		straight_with_avoidance(-300);
		goto_with_avoidance(sens * 175, 175, 0, CONTROL_BACKWARD);
		goto_with_avoidance(sens * 525, 175, 160, CONTROL_FORWARD);
		pince_open();
		straight_with_avoidance(-300);
	}
	else
	{
		goto_with_avoidance(sens * 175, 175, 0, CONTROL_BACKWARD);
	}

	control_set_use_us(US_BACK_MASK | US_FRONT_MASK);

	control_rotate(- sens * PI / 3);

	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
		
	
	// Scan de la table
	struct vect_pos pawn, pos;
	int pawn_taken = 0;
	vTaskDelay(ms_to_tick(400));
	
	if( detection_get_close_pawn(&pawn) == 0)
	{
		pos = odometry_get_position();
		if( distance_square(&pos, &pawn) < 1000*1000)
		{
			goto_with_avoidance(pawn.x, pawn.y, 160, CONTROL_FORWARD);
			pawn_taken = take_pawn();
		}
	}
	
	// Si on a trouvé et pris un pion, on le mets sur l'autre case à point
	if ( pawn_taken )
	{
		goto_with_avoidance(sens * 525, -525, -160, CONTROL_BACKWARD);
		pince_open();
		straight_with_avoidance(-300);
	}
	
	// pour finir on va essayer de choper un pion
	goto_with_avoidance(sens * 175, -525, 0, CONTROL_ANY_WAY);
	goto_with_avoidance(- sens * 525, -525, 160, CONTROL_FORWARD);
	if ( take_pawn() )
	{
		goto_with_avoidance( - sens * 175, -525, -160, CONTROL_BACKWARD);
	}
	
	pince_open();
}

static void strat_task()
{
	int sens = 1;
	pos_tour = 0;

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
	control_pince_dual(PINCE_POS_LOW, 0);

	action_first_pawn(sens);

	action_build_tower(sens);

	cambriolage_adversaire(sens);

	pince_open();

	while(1)
	{
		control_pince_independant(PINCE_POS_HI, PINCE_POS_LOW);
		vTaskDelay(ms_to_tick(400));
		control_pince_independant(PINCE_POS_LOW, PINCE_POS_HI);
		vTaskDelay(ms_to_tick(400));
	}

	vTaskDelete(NULL);
}

