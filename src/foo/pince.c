#include "pince.h"
#include "ax12.h"

#define PINCE_AX12_RIGHT      1
#define PINCE_AX12_LEFT       2

void pince_configure()
{
	// TODO ping des pinces // verif de la com
	ax12_set_moving_speed(PINCE_AX12_RIGHT, 0x3ff);
	ax12_set_moving_speed(PINCE_AX12_LEFT, 0x3ff);

	ax12_set_torque_limit(PINCE_AX12_RIGHT, 0x2ff);
	ax12_set_torque_limit(PINCE_AX12_LEFT, 0x2ff);

	ax12_set_torque_enable(PINCE_AX12_RIGHT, 1);
	ax12_set_torque_enable(PINCE_AX12_LEFT, 1);
}

void pince_open()
{
	ax12_set_goal_position(PINCE_AX12_RIGHT, 0x160);
	ax12_set_goal_position(PINCE_AX12_LEFT, 0x295);
}

void pince_close()
{
	ax12_set_goal_position(PINCE_AX12_RIGHT, 0x295);
	ax12_set_goal_position(PINCE_AX12_LEFT, 0x160);
}
