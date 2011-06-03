#include "pince.h"
#include "ax12.h"
#include "kernel/module.h"

#define PINCE_AX12_RIGHT      1
#define PINCE_AX12_LEFT       2

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
	ax12_set_goal_position(PINCE_AX12_RIGHT, 0x160);
	ax12_set_goal_position(PINCE_AX12_LEFT, 0x295);
	ax12_set_goal_position(PINCE_AX12_RIGHT, 0x160);
	ax12_set_goal_position(PINCE_AX12_LEFT, 0x295);
}

void pince_close()
{
	ax12_set_goal_position(PINCE_AX12_RIGHT, 0x295);
	ax12_set_goal_position(PINCE_AX12_LEFT, 0x160);
}

static void pince_module_exit()
{
	pince_open();
}

module_exit(pince_module_exit, EXIT_AX12);