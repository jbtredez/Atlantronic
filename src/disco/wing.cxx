#include <math.h>

#include "wing.h"
#include "kernel/driver/dynamixel.h"
#include "kernel/module.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"

static void wing_cmd(void* arg);

////////////////////////////////////////////////
/// function    : wing_module_init()
/// descrition  : Initialise Wing module
/// param       : none
/// retrun      : always 0
////////////////////////////////////////////////
static int wing_module_init()
{
	usb_add_cmd(USB_CMD_WING, &wing_cmd);

	// configuration des ax12
	ax12.set_torque_limit(AX12_LEFT_WING, 0.8);
	ax12.set_torque_limit(AX12_RIGHT_WING, 0.8);

	ax12.set_goal_limit(AX12_LEFT_WING, -M_PI_2, 0);
	ax12.set_goal_limit(AX12_RIGHT_WING, 0, M_PI_2);
/*
	ax12.set_moving_speed(AX12_LEFT_WING, DYNAMIXEL_MAX_MOVING_SPEED_RD);
	ax12.set_moving_speed(AX12_RIGHT_WING, DYNAMIXEL_MAX_MOVING_SPEED_RD);

	ax12.set_torque_enable(AX12_LEFT_WING, 1);
	ax12.set_torque_enable(AX12_RIGHT_WING, 1);
*/
	return 0;
}

module_init(wing_module_init, INIT_WING);



////////////////////////////////////////////////
/// function    : wing_set_position()
/// descrition  : Open Wings
/// param       : left = enum wing_cmd_type left wing state
/// param       : right = enum wing_cmd_type right wing state
/// retrun      : none
////////////////////////////////////////////////
void wing_set_position(enum wing_cmd_type left, enum wing_cmd_type right)
{
	switch(left)
	{
		case WING_PARK:
			ax12.set_goal_position(AX12_LEFT_WING, 0);
			break;
		case WING_OPEN:
			ax12.set_goal_position(AX12_LEFT_WING, -M_PI/2);
			break;
		default:
			break;
	}
	
	switch(right)
	{	
		case WING_PARK:
			ax12.set_goal_position(AX12_RIGHT_WING, 0);
			break;
		case WING_OPEN:
			ax12.set_goal_position(AX12_RIGHT_WING, M_PI/2);
			break;
		default:
			break;
	}
}

void wing_set_position(bool sym, enum wing_cmd_type left, enum wing_cmd_type right)
{
	if( sym )
	{
		wing_set_position(right, left);
	}
	else
	{
		wing_set_position(left, right);
	}
}

static void wing_cmd(void* arg)
{
	struct wing_cmd_arg* cmd_arg = (struct wing_cmd_arg*) arg;

	wing_set_position((enum wing_cmd_type)cmd_arg->type_left, (enum wing_cmd_type)cmd_arg->type_right);
}
