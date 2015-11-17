#include <math.h>

#include "kernel/module.h"
#include "finger.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"
#include "kernel/driver/Dynamixel.h"
#include "mainRobot.h"

static void finger_cmd(void* arg, void* data);

static int finger_module_init()
{
	// configuration des ax12
	lowFinger.setTorqueLimit(0.6);
	highFinger.setTorqueLimit(0.6);
	rightFinger.setTorqueLimit(0.6);
	leftFinger.setTorqueLimit(0.6);

	lowFinger.setGoalLimits(-1.8, 0);
	highFinger.setGoalLimits(0, 1.8);

	usb_add_cmd(USB_CMD_FINGER, finger_cmd, NULL);

	return 0;
}

static void finger_module_exit()
{
	finger_set_pos(FINGER_OPEN, FINGER_OPEN);
}

module_init(finger_module_init, INIT_FINGER);
module_exit(finger_module_exit, EXIT_FINGER);

void finger_set_pos(enum finger_type low, enum finger_type high)
{
	if( low > high)
	{
		high = low;
	}

	switch(low)
	{
		case FINGER_CLOSE:
			lowFinger.setGoalPosition(0);
			break;
		case FINGER_HALF_CLOSE:
			lowFinger.setGoalPosition(-22*M_PI/180);
			break;
		case FINGER_GOBLET:
			lowFinger.setGoalPosition(-60*M_PI/180);
			break;
		case FINGER_HALF_OPEN:
			lowFinger.setGoalPosition(-70*M_PI/180);
			break;
		case FINGER_OPEN:
			lowFinger.setGoalPosition(-M_PI_2);
			break;
		default:
			break;
	}

	switch(high)
	{
		case FINGER_CLOSE:
			highFinger.setGoalPosition(30*M_PI/180);
			break;
		case FINGER_HALF_CLOSE:
			highFinger.setGoalPosition(50*M_PI/180);
			break;
		case FINGER_GOBLET:
		case FINGER_HALF_OPEN:
			highFinger.setGoalPosition(60*M_PI/180);
			break;
		case FINGER_OPEN:
			highFinger.setGoalPosition(M_PI_2);
			break;
		default:
			break;
	}
}

void finger_bottom_set_pos(enum finger_bottom_type right, enum finger_bottom_type left)
{
	switch(right)
	{
		case FINGER_BOTTOM_CLOSE:
			rightFinger.setGoalPosition(100*M_PI/180);
			break;
		case FINGER_BOTTOM_OPEN:
			rightFinger.setGoalPosition(-60*M_PI/180);
			break;
		default:
			break;
	}

	switch(left)
	{
		case FINGER_BOTTOM_CLOSE:
			leftFinger.setGoalPosition(-100*M_PI/180);
			break;
		case FINGER_BOTTOM_OPEN:
			leftFinger.setGoalPosition(60*M_PI/180);
			break;
		default:
			break;
	}
}

static void finger_cmd(void* /*arg*/, void* data)
{
	struct finger_cmd_arg* cmd_arg = (struct finger_cmd_arg*) data;
	finger_set_pos((enum finger_type)cmd_arg->low, (enum finger_type)cmd_arg->high);
	finger_bottom_set_pos((enum finger_bottom_type)cmd_arg->right, (enum finger_bottom_type)cmd_arg->left);
}
