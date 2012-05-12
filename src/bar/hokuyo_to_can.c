#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "kernel/systick.h"
#include "kernel/event.h"
#include "kernel/driver/hokuyo.h"
#include "kernel/driver/can.h"
#include "kernel/can/can_id.h"
#include "kernel/hokuyo_tools.h"
#include "kernel/rcc.h"
#include "kernel/log.h"
#include "kernel/driver/usb.h"
#include "kernel/semphr.h"
#include "kernel/robot_parameters.h"
#include "kernel/math/trigo.h"
#include "gpio.h"
#include "location.h"
#include <math.h>


//! @todo réglage au pif
#define HOKUYO_TO_CAN_STACK_SIZE     400

static void hokuyo_to_can_task();
int hokuyo_to_can_module_init();
void hokuyo_to_can_compute();

static struct can_msg hokuyo_to_can_msg;

int hokuyo_to_can_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(hokuyo_to_can_task, "hoku_can", HOKUYO_TO_CAN_STACK_SIZE, NULL, PRIORITY_TASK_HOKUYO_TO_CAN, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_TEST;
	}

	hokuyo_to_can_msg.format = CAN_STANDARD_FORMAT;
	hokuyo_to_can_msg.type = CAN_DATA_FRAME;

	hokuyo_scan.sens =  PARAM_BAR_HOKUYO_SENS;
	hokuyo_scan.pos_hokuyo.x = PARAM_BAR_HOKUYO_X;
	hokuyo_scan.pos_hokuyo.y = PARAM_BAR_HOKUYO_Y;
	hokuyo_scan.pos_hokuyo.alpha = PARAM_BAR_HOKUYO_ALPHA;
	hokuyo_scan.pos_hokuyo.ca = fx_cos(hokuyo_scan.pos_hokuyo.alpha);
	hokuyo_scan.pos_hokuyo.sa = fx_sin(hokuyo_scan.pos_hokuyo.alpha);

	hokuyo_scan.pos_robot.x = 0;
	hokuyo_scan.pos_robot.y = 0;
	hokuyo_scan.pos_robot.alpha = 0;
	hokuyo_scan.pos_robot.ca = fx_cos(hokuyo_scan.pos_robot.alpha);
	hokuyo_scan.pos_robot.sa = fx_sin(hokuyo_scan.pos_robot.alpha);

	return 0;
}

module_init(hokuyo_to_can_module_init, INIT_DETECTION);

static void hokuyo_to_can_task()
{
	while(1)
	{
		// on attend la fin du nouveau scan
		vTaskWaitEvent(EVENT_LOCAL_HOKUYO_UPDATE, portMAX_DELAY);
		vTaskClearEvent(EVENT_LOCAL_HOKUYO_UPDATE);

		hokuyo_scan.pos_robot = location_get_position();

		xSemaphoreTake(hokuyo_scan_mutex, portMAX_DELAY);
		hokuyo_to_can_compute();
		xSemaphoreGive(hokuyo_scan_mutex);
	}

	vTaskDelete(NULL);
}

// TODO : calculer les segments et envoyer les seg
void hokuyo_to_can_compute()
{
#if 0
	unsigned int i = 0;
	int res;
	unsigned char* buf = (unsigned char*) hokuyo_scan.distance;

	// trame pour remettre le pointeur du tableau hokuyo à 0
	// c'est aussi utilisé pour avoir enregistrer la position du robot au moment du scan dans foo.
	hokuyo_to_can_msg.size = 0;
	hokuyo_to_can_msg.id = CAN_HOKUYO_DATA_RESET;

	res = can_write(&hokuyo_to_can_msg, ms_to_tick(50));
	if(res != 0)
	{
		// erreur : timeout sur le can, abandon
		log(LOG_ERROR, "timeout");
		goto end;
	}

	hokuyo_to_can_msg.id = CAN_HOKUYO_DATA;

	i = sizeof(hokuyo_scan.distance);
	for( ; i  ; )
	{
		if( i > 8)
		{
			hokuyo_to_can_msg.size = 8;
		}
		else
		{
			hokuyo_to_can_msg.size = i;
		}

		memcpy(hokuyo_to_can_msg.data, buf, hokuyo_to_can_msg.size);

		res = can_write(&hokuyo_to_can_msg, ms_to_tick(50));
		if(res != 0)
		{
			// erreur : timeout sur le can, abandon
			log(LOG_ERROR, "timeout");
			goto end;
		}

		i -= hokuyo_to_can_msg.size;
		buf +=  hokuyo_to_can_msg.size;
	}

end:
#endif
	return;
}
