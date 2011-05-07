//! @file can_us.c
//! @brief CAN - US
//! @author Atlantronic

//! interface : us.h

#include "kernel/us.h"
#include "kernel/driver/can.h"
#include "kernel/module.h"
#include "kernel/can_id.h"

void can_us_callback(struct can_msg *msg);

int can_us_module_init()
{
	can_register(CAN_US_ID, CAN_STANDARD_FORMAT, can_us_callback);

	return 0;
}

module_init(can_us_module_init, INIT_CAN_US);

void can_us_callback(struct can_msg *msg)
{

}
