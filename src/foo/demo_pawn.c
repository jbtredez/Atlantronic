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

static void demo_pawn_task();
int demo_pawn_module_init();

int demo_pawn_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(demo_pawn_task, "d_pawn", STRAT_STACK_SIZE, NULL, PRIORITY_TASK_STRATEGY, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_TEST;
	}

	return 0;
}

module_init(demo_pawn_module_init, INIT_STRATEGY);

static void demo_pawn_task()
{
	struct vect_pos pawn;
	struct vect_pos pawn2;
	struct vect_pos pos;

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

	pince_open();

	vTaskDelay(ms_to_tick(200));

	uint32_t event;
	int notFound = 0;
	int collision = 0;

	while(1)
	{
		if( detection_get_close_pawn(&pawn) == 0)
		{
			notFound = 0;
			collision = 1;
			pos = odometry_get_position();
			if( distance_square(&pos, &pawn) < 1500*1500)
			{
				do
				{
					if( collision )
					{
						control_goto_near(pawn.x, pawn.y, 160, CONTROL_FORWARD);
					}
					event = vTaskWaitEvent(EVENT_CONTROL_READY, ms_to_tick(200));
					if( distance_square(&pos, &pawn) > 600*600 )
					{
						if(detection_get_close_pawn(&pawn2) == 0)
						{
							if( distance_square(&pawn, &pawn2) > 100*100 )
							{
								control_free();
								notFound = 1;
							}
						}
						else
						{
							control_free();
							notFound = 1;
						}
					}

					collision = (event & EVENT_CONTROL_COLSISION) && (event & EVENT_CONTROL_READY);
				}while( collision || !(event & EVENT_CONTROL_READY));

				if( notFound == 0 )
				{
					pince_close();
					vTaskDelay(ms_to_tick(200));
					if( pince_full() )
					{					
						vTaskDelay(ms_to_tick(3000));
					}
					pince_open();
					vTaskDelay(ms_to_tick(3000));
				}
			}
		}

		vTaskDelay(ms_to_tick(100));
	}

	vTaskDelete(NULL);
}

