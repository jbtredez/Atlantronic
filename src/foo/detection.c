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
#include "kernel/driver/can.h"
#include "kernel/can/can_id.h"
#include "kernel/robot_parameters.h"
#include "location/location.h"
#include "gpio.h"

#include <math.h>

//! @todo réglage au pif
#define DETECTION_STACK_SIZE         400
#define HOKUYO_NUM_OBJECT            100
#define HOKUYO_FOO                     0
#define HOKUYO_BAR                     1


static void detection_task();
int detection_module_init();
void detection_compute();
void can_hokuyo_reset(struct can_msg *msg);
void can_hokuyo_data(struct can_msg *msg);

static int detection_can_hokuyo_id;
static struct hokuyo_scan hokuyo_scan[2];
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

	// TODO conf a virer dans kernel/robot_parameters.h
	hokuyo_scan[HOKUYO_FOO].sens = -1;
	hokuyo_scan[HOKUYO_FOO].pos_hokuyo.x = 60;
	hokuyo_scan[HOKUYO_FOO].pos_hokuyo.y = 60;
	hokuyo_scan[HOKUYO_FOO].pos_hokuyo.alpha = PI/4.0f;
	hokuyo_scan[HOKUYO_FOO].pos_hokuyo.ca = cosf(hokuyo_scan[HOKUYO_FOO].pos_hokuyo.alpha);
	hokuyo_scan[HOKUYO_FOO].pos_hokuyo.sa = sinf(hokuyo_scan[HOKUYO_FOO].pos_hokuyo.alpha);
	hokuyo_scan[HOKUYO_BAR].sens =  1;
	hokuyo_scan[HOKUYO_FOO].pos_hokuyo.x = 60;
	hokuyo_scan[HOKUYO_FOO].pos_hokuyo.y = -60;
	hokuyo_scan[HOKUYO_BAR].pos_hokuyo.alpha = -PI/4.0f;
	hokuyo_scan[HOKUYO_BAR].pos_hokuyo.ca = cosf(hokuyo_scan[HOKUYO_BAR].pos_hokuyo.alpha);
	hokuyo_scan[HOKUYO_BAR].pos_hokuyo.sa = sinf(hokuyo_scan[HOKUYO_BAR].pos_hokuyo.alpha);

	detection_can_hokuyo_id = 0;
	can_register(CAN_HOKUYO_DATA_RESET, CAN_STANDARD_FORMAT, can_hokuyo_reset);
	can_register(CAN_HOKUYO_DATA, CAN_STANDARD_FORMAT, can_hokuyo_data);

	return 0;
}

module_init(detection_module_init, INIT_DETECTION);

void detection_errors(uint32_t err)
{
	error_check_update(ERR_HOKUYO_DISCONNECTED, err);
	error_check_update(ERR_HOKUYO_USART_FE, err);
	error_check_update(ERR_HOKUYO_USART_NE, err);
	error_check_update(ERR_HOKUYO_USART_ORE, err);
	error_check_update(ERR_HOKUYO_CHECK_CMD, err);
	error_check_update(ERR_HOKUYO_UNKNOWN_STATUS, err);
	error_check_update(ERR_HOKUYO_CHECKSUM, err);
	error_check_update(ERR_HOKUYO_BAUD_RATE, err);
	error_check_update(ERR_HOKUYO_LASER_MALFUNCTION, err);
	error_check_update(ERR_HOKUYO_SCAN_SIZE, err);
	error_check_update(ERR_HOKUYO_DISTANCE_BUFFER, err);
}

void detection_hokuyo_init()
{
	uint32_t err;

	log_info("Initialisation du hokuyo");

	do
	{
		err = hokuyo_init();
		detection_errors(err);
		if( err)
		{
			vTaskDelay(ms_to_tick(100));
		}
	} while(err);

	log_info("Lancement des scan hokuyo");
}

static void detection_task()
{
	uint32_t err;
	portTickType last_scan_time;
	portTickType current_time;

	detection_hokuyo_init();

	hokuyo_start_scan();
	// on gruge, le premier scan est plus long
	last_scan_time = systick_get_time() + ms_to_tick(100);

	while(1)
	{
		// on attend la fin du nouveau scan
		err = hokuto_wait_decode_scan(hokuyo_scan[0].distance, HOKUYO_NUM_POINTS);
		if(err)
		{
			error(err, ERROR_ACTIVE);
			if(err == ERR_HOKUYO_DISCONNECTED)
			{
				detection_hokuyo_init();
			}
			// on gruge, le premier scan est plus long
			last_scan_time = systick_get_time() + ms_to_tick(100);
		}
		else
		{
			// on a un scan toutes les 100ms, ce qui laisse 100ms pour faire le calcul sur l'ancien scan
			// pendant que le nouveau arrive. Si on depasse les 110ms (10% d'erreur), on met un log
			current_time = systick_get_time();
			if( current_time - last_scan_time > ms_to_tick(110) )
			{
				log_error("slow cycle : %lu us", (long unsigned int) tick_to_us(current_time - last_scan_time));
			}
			last_scan_time = current_time;
		}

		// position mise en fin de scan
		hokuyo_scan[HOKUYO_FOO].pos_robot = location_get_position();

		// on lance le prochain scan avant de faire les calculs sur le scan actuel
		hokuyo_start_scan();

		// si le dernier scan n'a pas echoue on fait les calculs
		if( ! err)
		{
			detection_compute();

			// on envoi les donnees par usb pour le debug
			usb_add(USB_HOKUYO_FOO, &hokuyo_scan[HOKUYO_FOO], sizeof(hokuyo_scan[HOKUYO_FOO]));
		}
	}

	vTaskDelete(NULL);
}

void detection_compute()
{
	hokuyo_num_obj = hokuyo_find_objects(hokuyo_scan[HOKUYO_FOO].distance, HOKUYO_NUM_POINTS, hokuyo_object, HOKUYO_NUM_OBJECT);

//	hokuyo_compute_xy(hokuyo_distance, HOKUYO_NUM_POINTS, hokuyo_x, hokuyo_y, -1);
}

void can_hokuyo_reset(struct can_msg *msg)
{
	detection_can_hokuyo_id = 0;
	hokuyo_scan[HOKUYO_BAR].pos_robot = location_get_position();
}

void can_hokuyo_data(struct can_msg *msg)
{
	memcpy(((unsigned char*)hokuyo_scan[1].distance) + detection_can_hokuyo_id, msg->data, msg->size);
	detection_can_hokuyo_id += msg->size;
	if(detection_can_hokuyo_id == 1364)
	{
		usb_add(USB_HOKUYO_FOO_BAR, &hokuyo_scan[HOKUYO_BAR], sizeof(hokuyo_scan[HOKUYO_BAR]));
	}
}
