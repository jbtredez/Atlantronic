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

#define US_RIGHT_X (160)
#define US_RIGHT_Y (-160)
#define US_LEFT_X (160)
#define US_LEFT_Y (160)

static int us_module_init();
static void us_callback(struct can_msg *msg);
static uint16_t us_state[US_MAX];
static void us_task(void* arg);
static int us_module_init()
{
	can_register(CAN_US, CAN_STANDARD_FORMAT, us_callback);

	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(us_task, "us", US_STACK_SIZE, NULL, PRIORITY_TASK_CONTROL, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_CAN_US;
	}

	return 0;
}

module_init(us_module_init, INIT_CAN_US);

#if 0
static struct vect_pos US_get_spotted_point(int32_t us_source, struct vect_pos robot_pos)
{
	uint16_t us_distance;
	struct vect_pos point;

	if(us_source & US_LEFT_MASK )
	{
		us_distance = us_state[US_LEFT];
		/* Calcul de la position du point visé par les capteurs dans le repère du terrain */
		point.x = robot_pos.x + US_LEFT_X * robot_pos.ca - (us_distance + US_LEFT_Y) * robot_pos.sa;
		point.y = robot_pos.y + US_LEFT_X * robot_pos.sa + (us_distance + US_LEFT_Y) * robot_pos.ca;
	}
	else //if(us_scan & US_RIGHT_MASK)
	{
		us_distance = us_state[US_RIGHT];
		/* Calcul de la position du point visé par les capteurs dans le repère du terrain */
		point.x = robot_pos.x + US_RIGHT_X * robot_pos.ca - (US_RIGHT_Y-us_distance ) * robot_pos.sa;
		point.y = robot_pos.y + US_RIGHT_X * robot_pos.sa + (US_RIGHT_Y-us_distance ) * robot_pos.ca;
	}
	return point;
}
#endif

static void us_task(void* arg)
{
	(void) arg;
	while(1)
	{
#if 0
		if( us_scan & US_LEFT_MASK || us_scan & US_RIGHT_MASK)
		{
			struct vect_pos pos = location_get_position();
			struct vect_pos target = US_get_spotted_point(us_scan, pos);

			/* on vérifie que le x est dans une zone verte */
			if(   ((target.x > (-1400)) && (target.x < (-1400 + US_LONGEUR_CASE)))
			   || ((target.x < (1400)) && (target.x > (1400 - US_LONGEUR_CASE))))
			{

			}
		}
#endif
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
