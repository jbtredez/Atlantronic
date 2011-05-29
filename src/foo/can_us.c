//! @file can_us.c
//! @brief CAN - US
//! @author Atlantronic

//! interface : us.h

#include "kernel/module.h"
///include all files for can third party
#include "kernel/driver/can.h"
#include "kernel/can/can_id.h"
#include "kernel/can/can_us.h"

#include "kernel/portmacro.h"
#include <string.h>

uint16_t can_us_state[US_MAX];

void can_us_callback(struct can_msg *msg);

int can_us_module_init()
{
	can_register(CAN_US_EMERGENCY_ID, CAN_STANDARD_FORMAT, can_us_callback);

	return 0;
}

module_init(can_us_module_init, INIT_CAN_US);

//emergency function management
void can_us_callback(struct can_msg *msg)
{
	if(msg->data[0] < US_MAX && msg->size == 5)
	{
		portENTER_CRITICAL();
		memcpy(&can_us_state, msg->data + 1, 2);
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
		res = can_us_state[id];
		portEXIT_CRITICAL();
	}

	return res;
}