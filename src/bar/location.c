//! @file location.c
//! @brief Location
//! @author Atlantronic

#include "bar/location.h"
#include "kernel/module.h"
#include "kernel/portmacro.h"
#include "kernel/math/trigo.h"
#include "kernel/driver/can.h"
#include "kernel/can/can_id.h"

static struct kinematics location_kinematics;

static void location_can_kinematics1(struct can_msg *msg);
static void location_can_kinematics2(struct can_msg *msg);
static void location_can_kinematics3(struct can_msg *msg);

static int location_module_init()
{
	location_kinematics.x = 0;
	location_kinematics.y = 0;
	location_kinematics.alpha = 0;
	location_kinematics.ca = 1 << 30;
	location_kinematics.sa = 0;
	location_kinematics.v = 0;
	location_kinematics.w = 0;

	can_register(CAN_KINEMATICS_1, CAN_STANDARD_FORMAT, location_can_kinematics1);
	can_register(CAN_KINEMATICS_2, CAN_STANDARD_FORMAT, location_can_kinematics2);
	can_register(CAN_KINEMATICS_3, CAN_STANDARD_FORMAT, location_can_kinematics3);

	return 0;
};

module_init(location_module_init, INIT_LOCATION);

void location_can_kinematics1(struct can_msg *msg)
{
	portENTER_CRITICAL();
	location_kinematics.x = msg->_data.low;
	location_kinematics.y = msg->_data.high;
	portEXIT_CRITICAL();
}

void location_can_kinematics2(struct can_msg *msg)
{
	portENTER_CRITICAL();
	location_kinematics.alpha = msg->_data.low;
	location_kinematics.v = msg->_data.high;
	location_kinematics.ca = fx_cos(location_kinematics.alpha);
	location_kinematics.sa = fx_sin(location_kinematics.alpha);
	portEXIT_CRITICAL();
}

void location_can_kinematics3(struct can_msg *msg)
{
	portENTER_CRITICAL();
	location_kinematics.w = msg->_data.low;
	portEXIT_CRITICAL();
}

struct kinematics location_get_kinematics()
{
	struct kinematics k;
	portENTER_CRITICAL();
	k = location_kinematics;
	portEXIT_CRITICAL();
	return k;
}