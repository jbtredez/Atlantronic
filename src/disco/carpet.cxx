#include <math.h>

#include "kernel/module.h"
#include "carpet.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"
#include "kernel/driver/Dynamixel.h"
#include "disco/star.h"

static void carpet_cmd(void* arg, void* data);

static int carpet_module_init()
{
	// configuration des ax12
	leftCarpet.setTorqueLimit(1);
	rightCarpet.setTorqueLimit(1);

	leftCarpet.setGoalLimits(0, 1.4);
	rightCarpet.setGoalLimits(-1.4, 0);

	usb_add_cmd(USB_CMD_CARPET, carpet_cmd, NULL);
	return 0;
}

module_init(carpet_module_init, INIT_FINGER);

void carpet_set_pos(enum carpet_type right, enum carpet_type left)
{
	switch(right)
	{
		case CARPET_UP:
			rightCarpet.setGoalPosition(0);
			break;
		case CARPET_DOWN:
			rightCarpet.setGoalPosition(-1);
			break;
		case CARPET_NO_MOVE:
		default:
			break;
	}

	switch(left)
	{
		case CARPET_UP:
			leftCarpet.setGoalPosition(0);
			break;
		case CARPET_DOWN:
			leftCarpet.setGoalPosition(1);
			break;
		case CARPET_NO_MOVE:
		default:
			break;
	}
}

static void carpet_cmd(void* /*arg*/, void* data)
{
	struct carpet_cmd_arg* cmd_arg = (struct carpet_cmd_arg*) data;
	carpet_set_pos((enum carpet_type)cmd_arg->right, (enum carpet_type)cmd_arg->left);
}
