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

#define STRAT_STACK_SIZE       100

static void strat_task();
int strat_module_init();

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

void goto_with_avoidance(float x, float y, float delta)
{
	uint32_t event;
	do
	{
		control_goto_near(x, y, delta);
		event = vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	}while(event & EVENT_CONTROL_COLSISION);
}

void straight_with_avoidance(float dx)
{
	uint32_t event;
	do
	{
		control_straight(dx);
		event = vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);
	}while(event & EVENT_CONTROL_COLSISION);
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
	control_pince_dual(PINCE_POS_LOW, 0);

	vTaskWaitEvent(EVENT_HOKUYO_READY, portMAX_DELAY);

	// on avance sur la sortie de la case depart
	goto_with_avoidance(- sens * 1100, -850, 0);

	// si on a detecté un pion au lancement, on va le prendre
	if( is_pawn_front_start() )
	{
		goto_with_avoidance(- sens * 700, -700, 140);	
		pince_close();
		control_pince_dual(PINCE_POS_HI, 0);
		vTaskDelay(ms_to_tick(300));
		goto_with_avoidance(- sens * 700, -700, 0);
		goto_with_avoidance(- sens * 700, -750, 0);
		if(getcolor() == COLOR_BLUE)
		{
			us_start_scan(US_LEFT_MASK);
		}
		else
		{
			us_start_scan(US_RIGHT_MASK);
		}
	}
	else
	{
		goto_with_avoidance(- sens * 700, -700, 0);
		goto_with_avoidance(- sens * 700, -750, 0);
		if(getcolor() == COLOR_BLUE)
		{
			us_start_scan(US_LEFT_MASK);
		}
		else
		{
			us_start_scan(US_RIGHT_MASK);
		}
		vTaskDelay(ms_to_tick(400));
		float dist = get_distance();
		if( dist > 0)
		{
			dist += 60;
			float pos_pawn = -750 + dist + 100;
			if( fabsf(pos_pawn) < fabsf(pos_pawn + 350) )
			{
				goto_with_avoidance(- sens * 700, 0, 160);
			}
			else
			{
				goto_with_avoidance(- sens * 700, -350, 160);
			}
		}
		else
		{
			goto_with_avoidance(- sens * 700, 0, 160);
		}
		pince_close();
		control_pince_dual(PINCE_POS_HI, 0);
		vTaskDelay(ms_to_tick(300));
	}


	goto_with_avoidance(- sens * 700, 350, 160);
	goto_with_avoidance(- sens * 875, 830, 160);

	straight_with_avoidance(-300);

	goto_with_avoidance(- sens * 525, 525, 160);
	pince_open();

	straight_with_avoidance(-250);
	int pos_tour = us_get_scan_result();

	control_pince_dual(PINCE_POS_LOW, 0);

	control_rotate(sens * 3*PI/2.0f);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);

	switch(pos_tour)
	{
		case 0:
			goto_with_avoidance(- sens * 700, -350, 0);
			goto_with_avoidance(- sens * 1330, -350, 160);
			break;
		case 1:
			goto_with_avoidance(- sens * 700, -80, 0);
			goto_with_avoidance(- sens * 1330, -80, 0);
			break;
		case 2:
			goto_with_avoidance(- sens * 700, 200, 0);
			goto_with_avoidance(- sens * 1330, 200, 0);
			break;
		case 3:
			goto_with_avoidance(- sens * 700, 480, 0);
			goto_with_avoidance(- sens * 1330, 480, 0);
			break;
	}

	pince_close();
	vTaskDelay(ms_to_tick(400));
	control_pince_dual(PINCE_POS_HI, 0);
	straight_with_avoidance(-250);

	control_rotate(-sens*PI/2.0f);
	vTaskWaitEvent(EVENT_CONTROL_READY, portMAX_DELAY);

	goto_with_avoidance(- sens * 525, 525, 160);

	pince_open();

	vTaskDelete(NULL);
}

