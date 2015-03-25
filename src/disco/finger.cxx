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
	ax12.set_torque_limit(AX12_LOW_FINGER, 0.8);
	ax12.set_torque_limit(AX12_HIGH_FINGER, 0.8);

	ax12.set_goal_limit(AX12_LOW_FINGER, 0, M_PI_2);
	ax12.set_goal_limit(AX12_HIGH_FINGER, 0, M_PI_2);

	usb_add_cmd(USB_CMD_FINGER, finger_cmd);
	return 0;
}

module_init(finger_module_init, INIT_FINGER);

void finger_set_pos(enum finger_type low, enum finger_type high)
{
	switch(low)
	{
		case FINGER_CLOSE:
			ax12.set_goal_position(AX12_LOW_FINGER, 0);
			break;
		case FINGER_OPEN:
			ax12.set_goal_position(AX12_LOW_FINGER, M_PI_2);
			break;
		default:
			break;
	}

	switch(high)
	{
		case FINGER_CLOSE:
			ax12.set_goal_position(AX12_HIGH_FINGER, 0);
			break;
		case FINGER_OPEN:
			ax12.set_goal_position(AX12_HIGH_FINGER, M_PI_2);
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
