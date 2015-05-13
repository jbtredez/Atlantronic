#include <math.h>

#include "kernel/module.h"
#include "finger.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"
#include "kernel/driver/dynamixel.h"

static void finger_cmd(void* arg);

static int finger_module_init()
{
	// configuration des ax12
	ax12.set_torque_limit(AX12_LOW_FINGER, 0.6);
	ax12.set_torque_limit(AX12_HIGH_FINGER, 0.6);

	ax12.set_goal_limit(AX12_LOW_FINGER, -1.8, 0);
	ax12.set_goal_limit(AX12_HIGH_FINGER, 0, 1.8);

	usb_add_cmd(USB_CMD_FINGER, finger_cmd);
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
			ax12.set_goal_position(AX12_LOW_FINGER, 0);
			break;
		case FINGER_HALF_CLOSE:
			ax12.set_goal_position(AX12_LOW_FINGER, -22*M_PI/180);
			break;
		case FINGER_GOBLET:
			ax12.set_goal_position(AX12_LOW_FINGER, -60*M_PI/180);
			break;
		case FINGER_HALF_OPEN:
			ax12.set_goal_position(AX12_LOW_FINGER, -70*M_PI/180);
			break;
		case FINGER_OPEN:
			ax12.set_goal_position(AX12_LOW_FINGER, -M_PI_2);
			break;
		case FINGER_WIDE_OPEN:
			ax12.set_goal_position(AX12_LOW_FINGER, -1.8);
			break;
		default:
			break;
	}

	switch(high)
	{
		case FINGER_CLOSE:
			ax12.set_goal_position(AX12_HIGH_FINGER, 30*M_PI/180);
			break;
		case FINGER_HALF_CLOSE:
			ax12.set_goal_position(AX12_HIGH_FINGER, 50*M_PI/180);
			break;
		case FINGER_GOBLET:
		case FINGER_HALF_OPEN:
			ax12.set_goal_position(AX12_HIGH_FINGER, 60*M_PI/180);
			break;
		case FINGER_OPEN:
			ax12.set_goal_position(AX12_HIGH_FINGER, M_PI_2);
			break;
		case FINGER_WIDE_OPEN:
			ax12.set_goal_position(AX12_HIGH_FINGER, 1.8);
			break;
		default:
			break;
	}
}

static void finger_cmd(void* arg)
{
	struct finger_cmd_arg* cmd_arg = (struct finger_cmd_arg*) arg;
	finger_set_pos((enum finger_type)cmd_arg->low, (enum finger_type)cmd_arg->high);
}
