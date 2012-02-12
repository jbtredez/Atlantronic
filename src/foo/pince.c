#include "pince.h"
#include "ax12.h"
#include "kernel/module.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"

#define PINCE_AX12_RIGHT      1
#define PINCE_AX12_LEFT       2

#define AX12_RIGHT_EMPTY_THRESHOLD   0x200

static void pince_cmd(void* arg);

static int pince_module_init()
{
	usb_add_cmd(USB_CMD_PINCE, &pince_cmd);

	return 0;
}

module_init(pince_module_init, INIT_PINCE);

void pince_configure()
{
	// TODO ping des pinces // verif de la com
	ax12_set_moving_speed(PINCE_AX12_RIGHT, 0x3ff);
	ax12_set_moving_speed(PINCE_AX12_LEFT, 0x3ff);

	ax12_set_torque_limit(PINCE_AX12_RIGHT, 0x300);
	ax12_set_torque_limit(PINCE_AX12_LEFT, 0x300);

	ax12_set_torque_enable(PINCE_AX12_RIGHT, 1);
	ax12_set_torque_enable(PINCE_AX12_LEFT, 1);
	ax12_write8(PINCE_AX12_RIGHT, AX12_ALARM_SHUTDOWN, 0x04);
	ax12_write8(PINCE_AX12_LEFT, AX12_ALARM_SHUTDOWN, 0x04);
}

void pince_open()
{
	log(LOG_INFO, "pince_open");
	ax12_set_goal_position(PINCE_AX12_RIGHT, 0x160);
	ax12_set_goal_position(PINCE_AX12_LEFT, 0x295);
}

void pince_close()
{
	log(LOG_INFO, "pince_close");
	ax12_set_goal_position(PINCE_AX12_RIGHT, 0x295);
	ax12_set_goal_position(PINCE_AX12_LEFT, 0x160);
}

int pince_full()
{
	uint16_t ax12_pos = ax12_get_position(PINCE_AX12_RIGHT);
	return ax12_pos < AX12_RIGHT_EMPTY_THRESHOLD;
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