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

static int us_module_init();
static void us_callback(struct can_msg *msg);
static uint16_t us_state[US_MAX];

static int us_module_init()
{
	can_register(CAN_US, CAN_STANDARD_FORMAT, us_callback);

	return 0;
}

module_init(us_module_init, INIT_CAN_US);

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
