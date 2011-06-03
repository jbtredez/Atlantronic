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

//! @todo réglage au pif
#define DETECTION_STACK_SIZE         100
#define HOKUYO_NUM_POINTS            682

static void detection_task();
int detection_module_init();

static uint16_t hokuyo_distance[HOKUYO_NUM_POINTS]; //!< distances des angles 44 à 725 du hokuyo
static float hokuyo_x[682]; //!< x des points 44 à 725
static float hokuyo_y[682]; //!< y des points 44 à 725

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
	struct vect_pos pos_robot;

	do
	{
		err = hokuyo_init();
		if( err)
		{
			error_raise(err);
		}
	} while(err);

	hoku_init_pion();

	vTaskWaitEvent(EVENT_GO, portMAX_DELAY);

	int i = 2;
	for( ; i-- ;)
	{
		err = hokuyo_scan();
		if( err)
		{
			// TODO : checksum qui marche pas
//			error_raise(err);
		}

		hokuyo_decode_distance(hokuyo_distance, HOKUYO_NUM_POINTS);

		hokuyo_compute_xy(hokuyo_distance, HOKUYO_NUM_POINTS, hokuyo_x, hokuyo_y);

		hoku_init_tab(hokuyo_distance, 682, hokuyo_x, hokuyo_y);
		//hoku_parse_tab();
		parse_before_match_tab();
		
		vTaskDelay(ms_to_tick(100));
	}

	vTaskSetEvent(EVENT_HOKUYO_READY);

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

//		hokuyo_compute_xy(hokuyo_distance, HOKUYO_NUM_POINTS, hokuyo_x, hokuyo_y);

//		hoku_init_tab(hokuyo_distance, 682, hokuyo_x, hokuyo_y);

		//vérifie les anciens points
//		hoku_pion_table_verify_pawn(&pos_robot);
		//rajoute les nouveaux points (voir peut etre certains effacés)
//		hoku_parse_tab(&pos_robot);

		vTaskDelay(ms_to_tick(100));
	}

	vTaskDelete(NULL);
}

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
