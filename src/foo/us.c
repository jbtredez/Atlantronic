//! @file us.c
//! @brief Gestion des us
//! @author Atlantronic

#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "control/control.h"
#include "kernel/systick.h"
#include "kernel/event.h"
#include "gpio.h"
#include "location/location.h"
#include "kernel/rcc.h"
#include "kernel/driver/can.h"
#include "kernel/can/can_id.h"
#include "us.h"

#include "kernel/portmacro.h"
#include <string.h>

#define US_STACK_SIZE            100
#define US_SEUIL_FIGURE          550
#define US_SCAN_OFFSET           160
#define US_LARGEUR_CASE          80

static int us_module_init();
static void us_callback(struct can_msg *msg);
static uint16_t us_state[US_MAX];
static void us_task(void* arg);
static uint32_t scan[5];
static volatile int us_scan = 0;
static int us_module_init()
{
	can_register(CAN_US, CAN_STANDARD_FORMAT, us_callback);

	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(us_task, "us", US_STACK_SIZE, NULL, PRIORITY_TASK_CONTROL, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_CAN_US;
	}

	memset(scan, 0x00, 5);
	us_scan = 0;
	return 0;
}

module_init(us_module_init, INIT_CAN_US);

static void us_task(void* arg)
{
	(void) arg;
	while(1)
	{
		if( us_scan & US_LEFT_MASK || us_scan & US_RIGHT_MASK)
		{
			struct vect_pos pos = location_get_position();
			uint16_t val = us_state[US_LEFT];
			if( us_scan & US_RIGHT_MASK)
			{
				val = us_state[US_RIGHT];
			}

			pos.y += US_SCAN_OFFSET;
			if( val < US_SEUIL_FIGURE)
			{

				if( pos.y < -360 + US_LARGEUR_CASE && pos.y > -360 - US_LARGEUR_CASE)
				{
					scan[0]++;
				}
				if( pos.y < -80 + US_LARGEUR_CASE && pos.y > -80 - US_LARGEUR_CASE)
				{
					scan[1]++;
				}
				if( pos.y < 200 + US_LARGEUR_CASE && pos.y > 200 - US_LARGEUR_CASE)
				{
					scan[2]++;
				}
				if( pos.y < 480 + US_LARGEUR_CASE && pos.y > 480 - US_LARGEUR_CASE)
				{
					scan[3]++;
				}
				if( pos.y < 760 + US_LARGEUR_CASE && pos.y > 760 - US_LARGEUR_CASE)
				{
					scan[4]++;
				}
			}
		}

		vTaskDelay(ms_to_tick(5));
	}
}

uint8_t us_check_collision()
{
	uint8_t res = 0;

	portENTER_CRITICAL();
	if( us_state[US_FRONT] < 350 )
	{
		res |= US_FRONT_MASK;
	}

	if( us_state[US_BACK] < 200 )
	{
		res |= US_BACK_MASK;
	}
	portEXIT_CRITICAL();

	return res;
}

static void us_callback(struct can_msg *msg)
{
	if(msg->data[0] < US_MAX && msg->size == 5)
	{
		portENTER_CRITICAL();
		memcpy(&us_state[msg->data[0]], msg->data + 1, 2);
		portEXIT_CRITICAL();
	}
	else
	{
		// erreur TODO log
		// on fait rien, attente du prochain message CAN
	}
}

uint32_t us_get_state(enum us_id id)
{
	uint32_t res = 0;

	if(id < US_MAX)
	{
		portENTER_CRITICAL();
		res = us_state[id];
		portEXIT_CRITICAL();
	}

	return res;
}

void us_set_active(uint8_t us_acive_mask)
{
	struct can_msg msg;
	msg.type = CAN_DATA_FRAME;
	msg.format = CAN_STANDARD_FORMAT;
	msg.data[0] = us_acive_mask;
	msg.size = 1;
	msg.id = CAN_US_ACTIVATE;
	
	can_write(&msg, portMAX_DELAY);
}

// TODO pas de protection
void us_start_scan(uint8_t us_mask)
{
	memset(scan, 0x00, 5);
	if(us_mask & US_RIGHT_MASK)
	{
		us_set_active( US_RIGHT_MASK | US_FRONT_MASK | US_BACK_MASK);
	}
	else if(us_mask & US_LEFT_MASK)
	{
		us_set_active( US_LEFT_MASK  | US_FRONT_MASK | US_BACK_MASK);	
	}
	us_scan = us_mask;
}
