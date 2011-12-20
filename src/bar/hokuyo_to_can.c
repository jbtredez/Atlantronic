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

#include "gpio.h"
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
	portBASE_TYPE err = xTaskCreate(hokuyo_to_can_task, "detect", HOKUYO_TO_CAN_STACK_SIZE, NULL, PRIORITY_TASK_HOKUYO_TO_CAN, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_TEST;
	}

	hokuyo_to_can_msg.format = CAN_STANDARD_FORMAT;
	hokuyo_to_can_msg.type = CAN_DATA_FRAME;

	hokuyo_scan.sens =  1;
	hokuyo_scan.pos_robot.x = 0;
	hokuyo_scan.pos_robot.y = 0;
	hokuyo_scan.pos_robot.alpha = 0;
	hokuyo_scan.pos_robot.ca = 1;
	hokuyo_scan.pos_robot.sa = 0;
	hokuyo_scan.pos_hokuyo.x = 0;
	hokuyo_scan.pos_hokuyo.y = 0;
	hokuyo_scan.pos_hokuyo.alpha = 0;
	hokuyo_scan.pos_hokuyo.ca = 1;
	hokuyo_scan.pos_hokuyo.sa = 0;

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

		xSemaphoreTake(hokuyo_scan_mutex, portMAX_DELAY);
		hokuyo_to_can_compute();
		xSemaphoreGive(hokuyo_scan_mutex);
	}

	vTaskDelete(NULL);
}

void hokuyo_to_can_compute()
{
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
		log_error("timeout");
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
			log_error("timeout");
			goto end;
		}

		i -= hokuyo_to_can_msg.size;
		buf +=  hokuyo_to_can_msg.size;
	}

end:
	return;
}
