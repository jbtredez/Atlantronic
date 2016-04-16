#include <math.h>

#include "wing.h"
#include "kernel/driver/Dynamixel.h"
#include "kernel/module.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"
#include "disco/star/star.h"

static void wing_cmd(void* arg, void* data);

////////////////////////////////////////////////
/// function    : wing_module_init()
/// descrition  : Initialise Wing module
/// param       : none
/// retrun      : always 0
////////////////////////////////////////////////
static int wing_module_init()
{
	usb_add_cmd(USB_CMD_WING, &wing_cmd, NULL);

	// configuration des ax12
	leftWing.setTorqueLimit(0.8);
	rightWing.setTorqueLimit(0.8);

	//leftWing.setGoalLimits(-M_PI_2, 0);
	//rightWing.setGoalLimits(0, M_PI_2);

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
			leftWing.setGoalPosition(0);
			break;
		case WING_OPEN:
			leftWing.setGoalPosition(-M_PI/2);
			break;
		default:
			break;
	}
	
	switch(right)
	{	
		case WING_PARK:
			rightWing.setGoalPosition(0);
			break;
		case WING_OPEN:
			rightWing.setGoalPosition(M_PI/2);
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

static void wing_cmd(void* /*arg*/, void* data)
{
	struct wing_cmd_arg* cmd_arg = (struct wing_cmd_arg*) data;

	wing_set_position((enum wing_cmd_type)cmd_arg->type_left, (enum wing_cmd_type)cmd_arg->type_right);
}
