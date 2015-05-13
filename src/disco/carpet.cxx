#include <math.h>

#include "kernel/module.h"
#include "carpet.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"
#include "kernel/driver/dynamixel.h"

static void carpet_cmd(void* arg);

static int carpet_module_init()
{
	// configuration des ax12
	ax12.set_torque_limit(AX12_LEFT_CARPET, 1);
	ax12.set_torque_limit(AX12_RIGHT_CARPET, 1);

	ax12.set_goal_limit(AX12_LEFT_CARPET, 0, 1.4);
	ax12.set_goal_limit(AX12_RIGHT_CARPET, -1.4, 0);

	usb_add_cmd(USB_CMD_CARPET, carpet_cmd);
	return 0;
}

module_init(carpet_module_init, INIT_FINGER);

void carpet_set_pos(enum carpet_type right, enum carpet_type left)
{
	switch(right)
	{
		case CARPET_UP:
			ax12.set_goal_position(AX12_RIGHT_CARPET, 0);
			break;
		case CARPET_DOWN:
			ax12.set_goal_position(AX12_RIGHT_CARPET, -1.4);
			break;
		case CARPET_NO_MOVE:
		default:
			break;
	}

	switch(left)
	{
		case CARPET_UP:
			ax12.set_goal_position(AX12_LEFT_CARPET, 0);
			break;
		case CARPET_DOWN:
			ax12.set_goal_position(AX12_LEFT_CARPET, 1.4);
			break;
		case CARPET_NO_MOVE:
		default:
			break;
	}
}

static void carpet_cmd(void* arg)
{
	struct carpet_cmd_arg* cmd_arg = (struct carpet_cmd_arg*) arg;
	carpet_set_pos((enum carpet_type)cmd_arg->right, (enum carpet_type)cmd_arg->left);
}
