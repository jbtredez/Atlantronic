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

#include "gpio.h"
#include <math.h>


//! @todo réglage au pif
#define HOKUYO_TO_CAN_STACK_SIZE     400

static void hokuyo_to_can_task();
int hokuyo_to_can_module_init();
void hokuyo_to_can_compute();

static struct hokuyo_scan hokuyo_scan;
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

	return 0;
}

module_init(hokuyo_to_can_module_init, INIT_DETECTION);

static void hokuyo_to_can_task()
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
			error(err, ERROR_ACTIVE);
			vTaskDelay(ms_to_tick(100));
		}
	} while(err);

	log_info("Lancement des scan hokuyo");

	hokuyo_start_scan();
	last_scan_time = systick_get_time();

	while(1)
	{
		// on attend la fin du nouveau scan
		err = hokuto_wait_decode_scan(hokuyo_scan.distance, HOKUYO_NUM_POINTS);
		if(err)
		{
			error(err, ERROR_ACTIVE);
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
		hokuyo_start_scan();

		// si le dernier scan n'a pas echoue on fait les calculs
		if( ! err)
		{
			hokuyo_to_can_compute();

			// on envoi les donnees par usb pour le debug
			usb_add(USB_HOKUYO_BAR, &hokuyo_scan, sizeof(hokuyo_scan));
		}
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
