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
#include "kernel/can/can_us.h"

#include "kernel/portmacro.h"
#include <string.h>

#define US_STACK_SIZE            100

static void us_task();
static int us_module_init();
static void us_callback(struct can_msg *msg);
static uint16_t us_state[US_MAX];

static int us_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(us_task, "us", US_STACK_SIZE, NULL, PRIORITY_TASK_DETECTION, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_TEST;
	}

	can_register(CAN_US, CAN_STANDARD_FORMAT, us_callback);

	return 0;
}

module_init(us_module_init, INIT_CAN_US);

static void us_task()
{
	portTickType wake_time = systick_get_time();

	while(1)
	{
		/*if( us_state[US_FRONT] < 350 )
		{
			vTaskSetEvent(EVENT_US_COLLISION);
		}*/

		wake_time += ms_to_tick(5);
		vTaskDelayUntil(wake_time);
	}

	vTaskDelete(NULL);
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
