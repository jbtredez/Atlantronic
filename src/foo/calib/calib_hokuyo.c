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
#include <math.h>

#define CALIB_STACK_SIZE       300

struct hokuyo_scan calib_scan[2];
int16_t calib_distances[HOKUYO_NUM_POINTS];

static void calib_task();
int calib_module_init();

int calib_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(calib_task, "calib", CALIB_STACK_SIZE, NULL, PRIORITY_TASK_STRATEGY, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_TEST;
	}

	return 0;
}

module_init(calib_module_init, INIT_STRATEGY);

void calib_hokuyo(struct hokuyo_scan* scan)
{
	float d = 0;

	int i = 0;
	for( i = 0; i < HOKUYO_NUM_POINTS ; i++)
	{
		if( calib_scan[1].distance[i] > 20 && calib_scan[0].distance[i] > 20)
		{
			calib_distances[i] = (int16_t)calib_scan[1].distance[i] - (int16_t)calib_scan[0].distance[i];
		}
		else
		{
			calib_distances[i] = 0;
		}
	}
}

static void calib_task()
{
	vTaskWaitEvent(EVENT_GO, portMAX_DELAY);

	// on attend la fin du nouveau scan
	vTaskWaitEvent(EVENT_LOCAL_HOKUYO_UPDATE, portMAX_DELAY);
	vTaskClearEvent(EVENT_LOCAL_HOKUYO_UPDATE);

	xSemaphoreTake(hokuyo_scan_mutex, portMAX_DELAY);
	calib_scan[0] = hokuyo_scan;
	xSemaphoreGive(hokuyo_scan_mutex);
	calib_scan[0].pos_robot = location_get_position();

	control_straight(100.0f);

	// on attend la fin du nouveau scan
	vTaskWaitEvent(EVENT_LOCAL_HOKUYO_UPDATE, portMAX_DELAY);
	vTaskClearEvent(EVENT_LOCAL_HOKUYO_UPDATE);

	xSemaphoreTake(hokuyo_scan_mutex, portMAX_DELAY);
	calib_scan[1] = hokuyo_scan;
	xSemaphoreGive(hokuyo_scan_mutex);
	calib_scan[1].pos_robot = location_get_position();

	calib_hokuyo(calib_scan);

	vTaskDelete(NULL);
}