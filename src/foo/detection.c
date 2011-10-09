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
#include "kernel/log.h"

//! @todo réglage au pif
#define DETECTION_STACK_SIZE         400
#define HOKUYO_NUM_POINTS            682
#define HOKUYO_NUM_OBJECT            100
#define HOKUYO_NUM_PAWN               50

static void detection_task();
int detection_module_init();
uint32_t detection_compute();

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
	portTickType wake_time;
	portTickType current_time;

	do
	{
		log_info("Initialisation du hokuyo");
		err = hokuyo_init();
		if( err)
		{
			error_raise(err);
			log_error("hokuyo_init : error = %#.8x", (unsigned int)err);
			vTaskDelay(ms_to_tick(100));
		}
	} while(err);

	log_info("Lancement des scan hokuyo");

	// on doit avoir au moins 100ms entre 2 demandes de scan
	wake_time = systick_get_time();

	while(1)
	{
		err = detection_compute();
		if( err)
		{
			error_raise(err);
		}

		wake_time += ms_to_tick(150);
		current_time = systick_get_time();
		if( wake_time < current_time)
		{
			// on ne tiens pas le temps de cycle. Pb de com avec le hokuyo ou calcul trop long
			log_error("Tache detection retardee - delta = %d us", (unsigned int)tick_to_us(current_time - wake_time));
			wake_time = current_time + ms_to_tick(110);
		}

		vTaskDelayUntil(wake_time);
	}

	vTaskDelete(NULL);
}

uint32_t detection_compute()
{
	int i;
	struct vect_pos pos_robot;
	struct vect_pos pos_pawn;
	uint32_t err = 0;

	pos_robot = location_get_position();

	err = hokuyo_scan();
	if(err)
	{
		log_error("hokuyo_scan : err = %#.8x", (unsigned int)err);
		goto end;
	}

	hokuyo_decode_distance(hokuyo_distance, HOKUYO_NUM_POINTS);

#if 1
	hokuyo_num_obj = hokuyo_find_objects(hokuyo_distance, HOKUYO_NUM_POINTS, hokuyo_object, HOKUYO_NUM_OBJECT);

	portENTER_CRITICAL();
	for(i = 0, detection_num_pawn = 0; i < hokuyo_num_obj && detection_num_pawn < HOKUYO_NUM_PAWN ; i++)
	{
		if( hokuyo_object_is_pawn(hokuyo_distance, &hokuyo_object[i], &pos_pawn) )
		{
			// changement de repere hokuyo -> table
			pos_hokuyo_to_table(&pos_robot, &pos_pawn, &detection_pawn[detection_num_pawn]);
			detection_num_pawn++;
		}
	}
	portEXIT_CRITICAL();

//	hokuyo_compute_xy(hokuyo_distance, HOKUYO_NUM_POINTS, hokuyo_x, hokuyo_y, -1);
#endif
end:
	return err;
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
		if( (fabsf(detection_pawn[i].x + 700) < 100.0f && fabsf(detection_pawn[i].y + 700) < 100.0f) ||
		   ( fabsf(detection_pawn[i].x - 700) < 100.0f && fabsf(detection_pawn[i].y + 700) < 100.0f) )
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
