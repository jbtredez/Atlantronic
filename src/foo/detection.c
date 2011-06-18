#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "control/control.h"
#include "kernel/systick.h"
#include "kernel/event.h"
#include "kernel/driver/hokuyo.h"
#include "kernel/hokuyo_tools.h"
#include "gpio.h"
#include "location/location.h"
#include "kernel/rcc.h"
#include <math.h>

//! @todo réglage au pif
#define DETECTION_STACK_SIZE         100
#define HOKUYO_NUM_POINTS            682
#define HOKUYO_NUM_OBJECT            100
#define HOKUYO_NUM_PAWN               50

static void detection_task();
int detection_module_init();

static uint16_t hokuyo_distance[HOKUYO_NUM_POINTS]; //!< distances des angles 44 à 725 du hokuyo
//static float hokuyo_x[HOKUYO_NUM_POINTS]; //!< x des points 44 à 725
//static float hokuyo_y[HOKUYO_NUM_POINTS]; //!< y des points 44 à 725
static struct hokuyo_object hokuyo_object[HOKUYO_NUM_OBJECT];
static int hokuyo_num_obj;

static struct vect_pos detection_pawn[HOKUYO_NUM_PAWN];
static int detection_num_pawn;

int detection_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(detection_task, "detect", DETECTION_STACK_SIZE, NULL, PRIORITY_TASK_DETECTION, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_TEST;
	}

	return 0;
}

module_init(detection_module_init, INIT_DETECTION);

static void detection_task()
{
	uint32_t err;
	int i;
	struct vect_pos pos_robot;
	struct vect_pos pos_pawn;

	do
	{
		err = hokuyo_init();
		if( err)
		{
			error_raise(err);
		}
	} while(err);

	while(1)
	{
		pos_robot = location_get_position();
		err = hokuyo_scan();
		if( err)
		{
			// TODO : checksum qui marche pas
//			error_raise(err);
		}

		hokuyo_decode_distance(hokuyo_distance, HOKUYO_NUM_POINTS);

		hokuyo_num_obj = hokuyo_find_objects(hokuyo_distance, HOKUYO_NUM_POINTS, hokuyo_object, HOKUYO_NUM_OBJECT);

		portENTER_CRITICAL();
		for(i = 0, detection_num_pawn = 0; i < hokuyo_num_obj && detection_num_pawn < HOKUYO_NUM_PAWN ; i++)
		{
			if( hokuyo_object_is_pawn(hokuyo_distance, &hokuyo_object[i], &pos_pawn) )
			{
				// changement de repere hokuyo -> robot
				pos_pawn.x += 60;
				// changement de repere robot -> table
				pos_robot_to_table(&pos_robot, &pos_pawn, &detection_pawn[detection_num_pawn]);
				detection_num_pawn++;
			}
		}
		portEXIT_CRITICAL();

//		hokuyo_compute_xy(hokuyo_distance, HOKUYO_NUM_POINTS, hokuyo_x, hokuyo_y, -1);

		vTaskDelay(ms_to_tick(100));
	}

	vTaskDelete(NULL);
}

int detection_get_close_pawn(struct vect_pos *best_pawn)
{
	int res = 0;
	int i = 0;
	float best_dist2;
	float dist2;
	struct vect_pos pos_robot = location_get_position();


	portENTER_CRITICAL();
	if(detection_num_pawn == 0)
	{
		res = -1;
		goto end_critical;
	}

	best_dist2 = distance_square(&pos_robot, &detection_pawn[0]);
	*best_pawn = detection_pawn[0];

	i = 1;
	for( ; i < detection_num_pawn ; i++)
	{
		dist2 = distance_square(&pos_robot, &detection_pawn[i]);
		if( dist2 < best_dist2)
		{
			best_dist2 = dist2;
			*best_pawn = detection_pawn[i];
		}
	}

end_critical:
	portEXIT_CRITICAL();

	return res;
}


int is_pawn_front_start()
{
	int i = 0;
	int res = 0;

	portENTER_CRITICAL();
	for( ; i < detection_num_pawn ; i++)
	{
		if( (fabsf(detection_pawn[i].x + 700) < 100.0f && fabsf(detection_pawn[i].x + 700) < 100.0f) ||
		   ( fabsf(detection_pawn[i].x - 700) < 100.0f && fabsf(detection_pawn[i].x - 700) < 100.0f) )
		{
			res = 1;
			goto end_critical;
		}
	}

end_critical:
	portEXIT_CRITICAL();

	return res;
}

// TODO virer / hokuyo_tool
float get_distance()
{
	float dist = 0;
	int taille = 3;
	int i = 341 - taille;
	int n  = 0;
	for( ; i < 341 + taille ; i++)
	{
		if( hokuyo_distance[i] > 20 )
		{
			n++;
			dist += hokuyo_distance[i];
		}
	}

	if( n)
	{
		dist /= n;
	}

	return dist;
}
