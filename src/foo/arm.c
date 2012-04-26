//! @file arm.c
//! @brief Gestion du bras
//! @author Atlantronic

#include "arm.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/module.h"
#include "kernel/systick.h"
#include "kernel/rcc.h"
#include "ax12.h"

#define ARM_STACK_SIZE       300

static void arm_task();

static int arm_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(arm_task, "arm", ARM_STACK_SIZE, NULL, PRIORITY_TASK_ARM, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_ARM;
	}

	// TODO : mettre les limites
	ax12_set_goal_limit(AX12_ARM_1, 0, 0x3ff);
	ax12_set_goal_limit(AX12_ARM_2, 0, 0x3ff);

	// TODO gestion usb

	return 0;
}

module_init(arm_module_init, INIT_ARM);

static void arm_task()
{
	while(1)
	{
		vTaskDelay(ms_to_tick(1));
	}
}

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