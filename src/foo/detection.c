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
#include "kernel/math/regression.h"
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
static void detection_compute();
static void detection_compute_front_object();
void can_hokuyo_reset(struct can_msg *msg);
void can_hokuyo_data(struct can_msg *msg);

static int detection_can_hokuyo_id;
static struct hokuyo_scan hokuyo_scan_bar;
static struct hokuyo_object hokuyo_object[HOKUYO_NUM_OBJECT];
struct vect_pos detection_hokuyo_pos[HOKUYO_NUM_POINTS];
static int hokuyo_num_obj;
float detection_reg_ecart = 25;
static char detection_seg[HOKUYO_NUM_POINTS];
static struct vect_pos detection_front_object;

static xSemaphoreHandle detection_mutex;

int detection_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(detection_task, "detect", DETECTION_STACK_SIZE, NULL, PRIORITY_TASK_DETECTION, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_DETECTION;
	}

	detection_mutex = xSemaphoreCreateMutex();

	if(detection_mutex == NULL)
	{
		return ERR_INIT_DETECTION;
	}

	detection_front_object.x = 400000;
	detection_front_object.y = 400000;

	hokuyo_scan.sens = PARAM_FOO_HOKUYO_SENS;
	hokuyo_scan.pos_hokuyo.x = PARAM_FOO_HOKUYO_X;
	hokuyo_scan.pos_hokuyo.y = PARAM_FOO_HOKUYO_Y;
	hokuyo_scan.pos_hokuyo.alpha = PARAM_FOO_HOKUYO_ALPHA;
	hokuyo_scan.pos_hokuyo.ca = cosf(hokuyo_scan.pos_hokuyo.alpha);
	hokuyo_scan.pos_hokuyo.sa = sinf(hokuyo_scan.pos_hokuyo.alpha);
	hokuyo_scan_bar.sens =  PARAM_BAR_HOKUYO_SENS;
	hokuyo_scan_bar.pos_hokuyo.x = PARAM_BAR_HOKUYO_X;
	hokuyo_scan_bar.pos_hokuyo.y = PARAM_BAR_HOKUYO_Y;
	hokuyo_scan_bar.pos_hokuyo.alpha = PARAM_BAR_HOKUYO_ALPHA;
	hokuyo_scan_bar.pos_hokuyo.ca = cosf(hokuyo_scan_bar.pos_hokuyo.alpha);
	hokuyo_scan_bar.pos_hokuyo.sa = sinf(hokuyo_scan_bar.pos_hokuyo.alpha);

	hokuyo_precompute_angle(&hokuyo_scan, detection_hokuyo_pos);

	detection_can_hokuyo_id = 0;
	can_register(CAN_HOKUYO_DATA_RESET, CAN_STANDARD_FORMAT, can_hokuyo_reset);
	can_register(CAN_HOKUYO_DATA, CAN_STANDARD_FORMAT, can_hokuyo_data);

	return 0;
}

module_init(detection_module_init, INIT_DETECTION);

static void detection_task()
{
	while(1)
	{
		// on attend la fin du nouveau scan
		vTaskWaitEvent(EVENT_LOCAL_HOKUYO_UPDATE, portMAX_DELAY);
		vTaskClearEvent(EVENT_LOCAL_HOKUYO_UPDATE);

		// position mise en fin de scan
		//TODO demenager dans hokuyo ?
		hokuyo_scan.pos_robot = location_get_position();

		xSemaphoreTake(hokuyo_scan_mutex, portMAX_DELAY);
//		portTickType last_time = systick_get_time();
		detection_compute();
//		portTickType current_time = systick_get_time();
//		log_format(LOG_INFO, "compute_time : %lu us", (long unsigned int) tick_to_us(current_time - last_time));
		xSemaphoreGive(hokuyo_scan_mutex);
//		usb_add(USB_HOKUYO_FOO_SEG, &detection_seg, sizeof(detection_seg));
	}

	vTaskDelete(NULL);
}

void detection_get_front_object(struct vect_pos* obj)
{
	xSemaphoreTake(detection_mutex, portMAX_DELAY);
	*obj = detection_front_object;
	xSemaphoreGive(detection_mutex);
}

static void detection_compute_front_object()
{
	int i = 0;
	int a;
	while(i< HOKUYO_NUM_POINTS && detection_seg[i] != 1)
	{
		i++;
	}

	a = i;
	i++;

	float d;
	struct vect_pos obj = { 400000, 400000, 0, 1, 0 };
	for( ; i< HOKUYO_NUM_POINTS; i++)
	{
		if(detection_seg[i] == 1)
		{
			float dy = detection_hokuyo_pos[i].y - detection_hokuyo_pos[a].y;

			// elimination de segments
			if( (detection_hokuyo_pos[a].x < 0 && detection_hokuyo_pos[i].x < 0)
				|| (detection_hokuyo_pos[a].y > PARAM_LEFT_CORNER_Y && detection_hokuyo_pos[i].y > PARAM_LEFT_CORNER_Y)
				|| (detection_hokuyo_pos[a].y < PARAM_RIGHT_CORNER_Y && detection_hokuyo_pos[i].y < PARAM_RIGHT_CORNER_Y)
				|| (dy == 0) )
			{
				a = i;
				continue;
			}

			float dx = (detection_hokuyo_pos[i].x - detection_hokuyo_pos[a].x);

			float coef = (PARAM_RIGHT_CORNER_Y - detection_hokuyo_pos[a].y) / dy;
			if(coef < 0)
			{
				coef = 0;
			}
			else if(coef > 1)
			{
				coef = 1;
			}

			d = detection_hokuyo_pos[a].x + coef * dx;

			if( d < obj.x)
			{
				obj.x = d;
				obj.y = detection_hokuyo_pos[a].y + coef * (detection_hokuyo_pos[i].y - detection_hokuyo_pos[a].y);
			}

			coef = (PARAM_LEFT_CORNER_Y - detection_hokuyo_pos[a].y) / dy;
			if(coef < 0)
			{
				coef = 0;
			}
			else if(coef > 1)
			{
				coef = 1;
			}

			d = detection_hokuyo_pos[a].x + coef * dx;
			if( d < obj.x)
			{
				obj.x = d;
				obj.y = detection_hokuyo_pos[a].y + coef * (detection_hokuyo_pos[i].y - detection_hokuyo_pos[a].y);
			}

			a = i;
		}
	}

	if(obj.x < PARAM_LEFT_CORNER_X || obj.x < PARAM_RIGHT_CORNER_X)
	{
		// erreur de calibration des hokuyo (position en x dans le repère robot) ou de la position des coins (x)
		log_format(LOG_ERROR, "erreur de calibration ? : obj %.2f %.2f", obj.x, obj.y);
	}

	xSemaphoreTake(detection_mutex, portMAX_DELAY);
	pos_robot_to_table(&hokuyo_scan.pos_robot, &obj, &detection_front_object);
	xSemaphoreGive(detection_mutex);
}

void detection_compute()
{
//	hokuyo_num_obj = hokuyo_find_objects(hokuyo_scan.distance, HOKUYO_NUM_POINTS, hokuyo_object, HOKUYO_NUM_OBJECT);

	hokuyo_compute_xy(&hokuyo_scan, detection_hokuyo_pos);
	regression_poly(detection_hokuyo_pos, HOKUYO_NUM_POINTS, detection_reg_ecart, detection_seg);

	detection_compute_front_object();

	log_format(LOG_DEBUG2, "front object %.2f %.2f", detection_front_object.x, detection_front_object.y);
}

void can_hokuyo_reset(struct can_msg *msg)
{
	detection_can_hokuyo_id = 0;
	hokuyo_scan_bar.pos_robot = location_get_position();
}

void can_hokuyo_data(struct can_msg *msg)
{
	memcpy(((unsigned char*)hokuyo_scan_bar.distance) + detection_can_hokuyo_id, msg->data, msg->size);
	detection_can_hokuyo_id += msg->size;
	if(detection_can_hokuyo_id == 1364)
	{
		usb_add(USB_HOKUYO_FOO_BAR, &hokuyo_scan_bar, sizeof(hokuyo_scan_bar));
	}
}
