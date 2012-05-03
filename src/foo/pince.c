#include "pince.h"
#include "ax12.h"
#include "kernel/module.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"

#define AX12_RIGHT_EMPTY_THRESHOLD   0x200

static void pince_cmd(void* arg);

static int pince_module_init()
{
	usb_add_cmd(USB_CMD_PINCE, &pince_cmd);

	ax12_set_goal_limit(AX12_PINCE_RIGHT, 0x150, 0x320);
	ax12_set_goal_limit(AX12_PINCE_LEFT, 0xdf, 0x2af);

	return 0;
}

module_init(pince_module_init, INIT_PINCE);

void pince_configure()
{
	ax12_set_moving_speed(AX12_PINCE_RIGHT, 0x3ff);
	ax12_set_moving_speed(AX12_PINCE_LEFT, 0x3ff);

	ax12_set_torque_limit(AX12_PINCE_RIGHT, 0x300);
	ax12_set_torque_limit(AX12_PINCE_LEFT, 0x300);

	ax12_set_torque_enable(AX12_PINCE_RIGHT, 1);
	ax12_set_torque_enable(AX12_PINCE_LEFT, 1);
//	ax12_write8(AX12_PINCE_RIGHT, AX12_ALARM_SHUTDOWN, 0x04);
//	ax12_write8(AX12_PINCE_LEFT, AX12_ALARM_SHUTDOWN, 0x04);
}

// TODO tache de gestion des pinces
void pince_open()
{
	log(LOG_INFO, "pince_open");
	ax12_set_goal_position(AX12_PINCE_RIGHT, 15000000);
	ax12_set_goal_position(AX12_PINCE_LEFT, -15000000);
}

void pince_close()
{
	log(LOG_INFO, "pince_close");
	ax12_set_goal_position(AX12_PINCE_RIGHT, -15000000);
	ax12_set_goal_position(AX12_PINCE_LEFT,   15000000);
}

static void pince_cmd(void* arg)
{
	struct pince_cmd_arg* cmd_arg = (struct pince_cmd_arg*) arg;

	switch(cmd_arg->type)
	{
		case PINCE_CONFIGURE:
			pince_configure();
			break;
		case PINCE_OPEN:
			pince_open();
			break;
		case PINCE_CLOSE:
			pince_close();
			break;
		default:
			break;
	}
}