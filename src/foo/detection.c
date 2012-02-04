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
#include "kernel/math/segment_intersection.h"
#include "kernel/math/trigo.h"
#include "location/location.h"
#include "gpio.h"

//! @todo réglage au pif
#define DETECTION_STACK_SIZE         400
#define DETECTION_NUM_OBJECT         100
#define HOKUYO_REG_SEG               200
#define HOKUYO_FOO                     0
#define HOKUYO_BAR                     1


static void detection_task();
int detection_module_init();
static void detection_compute();
static void detection_compute_front_object(struct fx_vect_pos pos);
void can_hokuyo_reset(struct can_msg *msg);
void can_hokuyo_data(struct can_msg *msg);

static int detection_can_hokuyo_id;
static struct hokuyo_scan hokuyo_scan_bar;
static struct hokuyo_object detection_object[DETECTION_NUM_OBJECT];
struct fx_vect2 detection_hokuyo_pos[HOKUYO_NUM_POINTS];
struct fx16_vect2 detection_hokuyo_reg[HOKUYO_REG_SEG];
int detection_reg_size;
static int detection_num_obj;
int detection_reg_ecart = 25;

static struct fx_vect2 detection_front_seg[2];

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

	detection_reg_size = 0;

	detection_front_seg[0].x = 1 << 30;
	detection_front_seg[0].y = -1 << 30;
	detection_front_seg[1].x = 1 << 30;
	detection_front_seg[1].y = 1 << 30;

	hokuyo_scan.sens = PARAM_FOO_HOKUYO_SENS;
	hokuyo_scan.pos_hokuyo.x = PARAM_FOO_HOKUYO_X;
	hokuyo_scan.pos_hokuyo.y = PARAM_FOO_HOKUYO_Y;
	hokuyo_scan.pos_hokuyo.alpha = PARAM_FOO_HOKUYO_ALPHA;
	hokuyo_scan.pos_hokuyo.ca = fx_cos(hokuyo_scan.pos_hokuyo.alpha);
	hokuyo_scan.pos_hokuyo.sa = fx_sin(hokuyo_scan.pos_hokuyo.alpha);
	hokuyo_scan_bar.sens =  PARAM_BAR_HOKUYO_SENS;
	hokuyo_scan_bar.pos_hokuyo.x = PARAM_BAR_HOKUYO_X;
	hokuyo_scan_bar.pos_hokuyo.y = PARAM_BAR_HOKUYO_Y;
	hokuyo_scan_bar.pos_hokuyo.alpha = PARAM_BAR_HOKUYO_ALPHA;
	hokuyo_scan_bar.pos_hokuyo.ca = fx_cos(hokuyo_scan_bar.pos_hokuyo.alpha);
	hokuyo_scan_bar.pos_hokuyo.sa = fx_sin(hokuyo_scan_bar.pos_hokuyo.alpha);

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
		if( detection_reg_size )
		{
			usb_add(USB_HOKUYO_FOO_SEG, &detection_hokuyo_reg, sizeof(detection_hokuyo_reg[0]) * detection_reg_size);
		}
	}

	vTaskDelete(NULL);
}

void detection_get_front_seg(struct fx_vect2* a, struct fx_vect2* b)
{
	xSemaphoreTake(detection_mutex, portMAX_DELAY);
	*a = detection_front_seg[0];
	*b = detection_front_seg[1];
	xSemaphoreGive(detection_mutex);
}

static void detection_compute_front_object(struct fx_vect_pos pos)
{
	int i = 0;
	int j = 0;
	int32_t x_min = 1 << 30;
	int objId = -1;

	struct fx_vect2 c;
	struct fx_vect2 d;
	struct fx_vect2 h;
	struct fx_vect2 tmp;

	struct fx_vect2 a1 = { 0,  PARAM_LEFT_CORNER_Y };
	struct fx_vect2 b1 = { 1 << 30,  PARAM_LEFT_CORNER_Y };
	struct fx_vect2 a2 = { 0, PARAM_RIGHT_CORNER_Y };
	struct fx_vect2 b2 = { 1 << 30, PARAM_RIGHT_CORNER_Y };

	for( i = 0 ; i < detection_num_obj ; i++)
	{
		j = detection_object[i].start;
		int maxj = j + detection_object[i].size;
		tmp.x = ((int32_t) detection_hokuyo_reg[j].x) << 16;
		tmp.y = ((int32_t) detection_hokuyo_reg[j].y) << 16;
		fx_vect2_table_to_robot(&pos, &tmp, &c);
		for( j++; j < maxj; j++)
		{
			tmp.x = ((int32_t) detection_hokuyo_reg[j].x) << 16;
			tmp.y = ((int32_t) detection_hokuyo_reg[j].y) << 16;
			fx_vect2_table_to_robot(&pos, &tmp, &d);
			int err1 = segment_intersection(a1, b1, c, d, &h);
			if(err1 == 0)
			{
				if( h.x < x_min)
				{
					x_min = h.x;
					objId = j;
				}
			}

			int err2 = segment_intersection(a2, b2, c, d, &h);
			if(err2 == 0)
			{
				if( h.x < x_min)
				{
					x_min = h.x;
					objId = j;
				}
			}

			if( err1 && err2 )
			{
				// pas d'intersection avec [a1 b1] ou [a2 b2]
				// on regarde si le segment n'est pas completement entre les deux
				// inutle de verifier les 2 points vu qu'il n'y a pas d'intersection avec les 2 segments
				if( c.x > 0 && c.y > a1.y && c.y < a2.y)
				{
					if( c.x < x_min)
					{
						x_min = c.x;
						objId = j;
					}
					if( d.x < x_min)
					{
						x_min = c.x;
						objId = j;
					}
				}
			}

			c = d;
		}
	}

	if(x_min < PARAM_LEFT_CORNER_X || x_min < PARAM_RIGHT_CORNER_X)
	{
		// erreur de calibration des hokuyo (position en x dans le repère robot) ou de la position des coins (x)
		log_format(LOG_ERROR, "erreur de calibration ? : x_min %ld",x_min >> 16);
	}

	if(objId >= 0)
	{
		i = detection_object[objId].start;
		j = i + detection_object[objId].size-1;

		b1.x = x_min;
		b2.x = x_min;

		if( detection_hokuyo_reg[i].y < detection_hokuyo_reg[j].y)
		{
			b1.y = detection_hokuyo_reg[i].y;
			b2.y = detection_hokuyo_reg[j].y;
		}
		else
		{
			b1.y = detection_hokuyo_reg[j].y;
			b2.y = detection_hokuyo_reg[i].y;
		}
	}

	xSemaphoreTake(detection_mutex, portMAX_DELAY);
	fx_vect2_robot_to_table(&hokuyo_scan.pos_robot, &b1, detection_front_seg);
	fx_vect2_robot_to_table(&hokuyo_scan.pos_robot, &b2, detection_front_seg+1);
	xSemaphoreGive(detection_mutex);
}

void detection_compute()
{
	int i;
	detection_reg_size = 0;

	detection_num_obj = hokuyo_find_objects(hokuyo_scan.distance, HOKUYO_NUM_POINTS, detection_object, DETECTION_NUM_OBJECT);
	hokuyo_compute_xy(&hokuyo_scan, detection_hokuyo_pos);

	for( i = 0 ; i < detection_num_obj ; i++)
	{
		detection_object[i].size = regression_poly(detection_hokuyo_pos + detection_object[i].start, detection_object[i].size, detection_reg_ecart, detection_hokuyo_reg + detection_reg_size, HOKUYO_REG_SEG - detection_reg_size);
		detection_object[i].start = detection_reg_size;
		detection_reg_size += detection_object[i].size;
	}

	struct fx_vect_pos pos = location_get_position();
	detection_compute_front_object(pos);

	log_format(LOG_DEBUG2, "front object %d %d %d %d", (int)detection_front_seg[0].x >> 16, (int)detection_front_seg[0].y >> 16, (int)detection_front_seg[1].x >> 16, (int)detection_front_seg[1].y >> 16);
}

void can_hokuyo_reset(struct can_msg *msg)
{
	(void) msg;
	detection_can_hokuyo_id = 0;
//	hokuyo_scan_bar.pos_robot = location_get_position();
}

void can_hokuyo_data(struct can_msg *msg)
{
	memcpy(((unsigned char*)hokuyo_scan_bar.distance) + detection_can_hokuyo_id, msg->data, msg->size);
	detection_can_hokuyo_id += msg->size;
	if(detection_can_hokuyo_id == 1364)
	{
//		usb_add(USB_HOKUYO_FOO_BAR, &hokuyo_scan_bar, sizeof(hokuyo_scan_bar));
	}
}
