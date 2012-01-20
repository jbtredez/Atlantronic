#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "control/control.h"
#include "kernel/systick.h"
#include "kernel/event.h"
#include "location/location.h"
#include "kernel/rcc.h"
#include "kernel/robot_parameters.h"
#include "kernel/hokuyo_tools.h"
#include "kernel/driver/usb.h"
#include <math.h>
#include <stdlib.h>

#define CALIB_STACK_SIZE       300
#define HOKUYO_START_ANGLE               (-(135 / 180.0f - 44 / 512.0f) * PI)
#define HOKUYO_DTHETA         	         (PI / 512.0f)

struct hokuyo_scan calib_scan[2];

static void calib_task();
int calib_module_init();
int16_t calib_distance[HOKUYO_NUM_POINTS];
float calib_angle;
int calib_best_id;

int calib_module_init()
{
	calib_angle = 0;
	calib_best_id = 0;

	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(calib_task, "calib", CALIB_STACK_SIZE, NULL, PRIORITY_TASK_STRATEGY, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_TEST;
	}

	return 0;
}

module_init(calib_module_init, INIT_STRATEGY);

void calib_hokuyo()
{
	int i = 0;
	int best = 4096;
	calib_best_id = 0;
	for( i = 0; i < HOKUYO_NUM_POINTS ; i++)
	{
		if( calib_scan[1].distance[i] > 20 && calib_scan[0].distance[i] > 20)
		{
			calib_distance[i] = abs((int16_t)calib_scan[0].distance[i] - (int16_t)calib_scan[1].distance[i] - 600);
			if(calib_distance[i] < best)
			{
				best = calib_distance[i];
				calib_best_id = i;
			}
		}
		else
		{
			calib_distance[i] = 4096;
		}
	}
	calib_angle =  HOKUYO_START_ANGLE + calib_scan[0].sens * calib_scan[0].pos_hokuyo.alpha + HOKUYO_DTHETA * calib_best_id;
}

static void calib_task()
{
#if 0
	vTaskWaitEvent(EVENT_GO, portMAX_DELAY);

	// on attend la fin du nouveau scan
	vTaskWaitEvent(EVENT_LOCAL_HOKUYO_UPDATE, portMAX_DELAY);
	vTaskClearEvent(EVENT_LOCAL_HOKUYO_UPDATE);

	xSemaphoreTake(hokuyo_scan_mutex, portMAX_DELAY);
	calib_scan[0] = hokuyo_scan;
	xSemaphoreGive(hokuyo_scan_mutex);
	calib_scan[0].pos_robot = location_get_position();

	control_straight(600.0f);
	vTaskWaitEvent(EVENT_CONTROL_READY | EVENT_CONTROL_COLSISION, portMAX_DELAY);

	vTaskDelay(ms_to_tick(200));

	// on attend la fin du nouveau scan
	vTaskWaitEvent(EVENT_LOCAL_HOKUYO_UPDATE, portMAX_DELAY);
	vTaskClearEvent(EVENT_LOCAL_HOKUYO_UPDATE);

	xSemaphoreTake(hokuyo_scan_mutex, portMAX_DELAY);
	calib_scan[1] = hokuyo_scan;
	xSemaphoreGive(hokuyo_scan_mutex);
	calib_scan[1].pos_robot = location_get_position();

	calib_hokuyo();

	control_straight(-600.0f);
	vTaskWaitEvent(EVENT_CONTROL_READY | EVENT_CONTROL_COLSISION, portMAX_DELAY);
#endif
	vTaskDelete(NULL);
}