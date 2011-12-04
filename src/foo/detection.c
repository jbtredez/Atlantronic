#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "kernel/systick.h"
#include "kernel/event.h"
#include "kernel/driver/hokuyo.h"
#include "kernel/hokuyo_tools.h"
#include "kernel/rcc.h"
#include "kernel/log.h"
#include "kernel/driver/usb.h"
#include "location/location.h"
#include "gpio.h"

#include <math.h>

//! @todo réglage au pif
#define DETECTION_STACK_SIZE         400
#define HOKUYO_NUM_OBJECT            100

static void detection_task();
int detection_module_init();
void detection_compute();

static struct hokuyo_scan hokuyo_scan;
//static float hokuyo_x[HOKUYO_NUM_POINTS]; //!< x des points 44 à 725
//static float hokuyo_y[HOKUYO_NUM_POINTS]; //!< y des points 44 à 725
static struct hokuyo_object hokuyo_object[HOKUYO_NUM_OBJECT];
static int hokuyo_num_obj;

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
	portTickType last_scan_time;
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

	hokuyo_scan.pos = location_get_position();
	hokuyo_start_scan();
	last_scan_time = systick_get_time();

	while(1)
	{
		// on attend la fin du nouveau scan
		err = hokuto_wait_decode_scan(hokuyo_scan.distance, HOKUYO_NUM_POINTS);
		if(err)
		{
			error_raise(err);
			log_error("scan : err = %#.8x", (unsigned int)err);
		}

		// on a un scan toutes les 100ms, ce qui laisse 100ms pour faire le calcul sur l'ancien scan
		// pendant que le nouveau arrive. Si on depasse les 110ms (10% d'erreur), on met un log
		current_time = systick_get_time();
		if( current_time - last_scan_time > ms_to_tick(110))
		{
			log_error("slow cycle : %lu us", (long unsigned int) tick_to_us(current_time - last_scan_time));
		}
		last_scan_time = current_time;

		// on lance le prochain scan avant de faire les calculs sur le scan actuel
		hokuyo_scan.pos = location_get_position();
		hokuyo_start_scan();

		// si le dernier scan n'a pas echoue on fait les calculs
		if( ! err)
		{
			detection_compute();

			// on envoi les donnees par usb pour le debug
			usb_add(USB_HOKUYO, &hokuyo_scan, sizeof(hokuyo_scan));
		}
	}

	vTaskDelete(NULL);
}

void detection_compute()
{
	hokuyo_num_obj = hokuyo_find_objects(hokuyo_scan.distance, HOKUYO_NUM_POINTS, hokuyo_object, HOKUYO_NUM_OBJECT);

//	hokuyo_compute_xy(hokuyo_distance, HOKUYO_NUM_POINTS, hokuyo_x, hokuyo_y, -1);
}
