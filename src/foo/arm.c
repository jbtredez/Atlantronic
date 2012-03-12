//! @file arm.c
//! @brief Gestion du bras
//! @author Atlantronic

#include "arm.h"
#include "kernel/module.h"
#include "ax12.h"

static int arm_module_init()
{
	// TODO gestion usb

	return 0;
}

module_init(arm_module_init, INIT_ARM);

int arm_goto_zab(int32_t z, int32_t a, int32_t b)
{
	// TODO axe z
	ax12_set_goal_position(AX12_ARM_1, a);
	ax12_set_goal_position(AX12_ARM_2, b);

	return 0;
}

int arm_goto_xyz(int32_t x, int32_t y, int32_t z)
{
	// TODO changement repère robot vers repere articulaire
	int32_t a = 0;
	int32_t b = 0;

	arm_goto_zab(z, a, b);

	return 0;
}