//! @file arm.c
//! @brief Gestion du bras
//! @author Atlantronic

#include "arm.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "kernel/systick.h"
#include "kernel/rcc.h"
#include "kernel/log.h"
#include "kernel/trapeze.h"
#include "kernel/driver/usb.h"
#include "foo/pwm.h"
#include "ax12.h"

#define ARM_STACK_SIZE       300
#define ARM_STEP_BY_MM         5
#define ARM_HZ              1000
#define ARM_ZMAX            (200 << 16)

static xSemaphoreHandle arm_mutex;
static uint32_t arm_z_cmd;    //!< commande en hauteur du bras

static int32_t arm_z;
static int32_t arm_vz;
static int32_t arm_z_step;

const int32_t ARM_VMAX = (125 << 16) / ARM_HZ;
const int32_t ARM_AMAX = (400 << 16) / (ARM_HZ * ARM_HZ);
const int32_t ARM_DMAX = (400 << 16) / (ARM_HZ * ARM_HZ);

static void arm_cmd_zab(void* arg);
static void arm_cmd_bridge(void* arg);
static void arm_task();

static int arm_module_init()
{
	xTaskHandle xHandle;
	portBASE_TYPE err = xTaskCreate(arm_task, "arm", ARM_STACK_SIZE, NULL, PRIORITY_TASK_ARM, &xHandle);

	if(err != pdPASS)
	{
		return ERR_INIT_ARM;
	}

	arm_mutex = xSemaphoreCreateMutex();

	if(arm_mutex == NULL)
	{
		return ERR_INIT_ARM;
	}

	// TODO : mettre les limites
	ax12_set_goal_limit(AX12_ARM_1, 0, 0x3ff);
	ax12_set_goal_limit(AX12_ARM_2, 0, 0x3ff);

	usb_add_cmd(USB_CMD_ARM, &arm_cmd_zab);
	usb_add_cmd(USB_CMD_ARM_BRIDGE, &arm_cmd_bridge);

	return 0;
}

module_init(arm_module_init, INIT_ARM);

static void arm_task()
{
	// TODO procedure de mise à zéro
	arm_z = 0;
	arm_vz = 0;
	arm_z_step = 0;

	GPIOB->ODR |= GPIO_ODR_ODR12;

	while(1)
	{
		arm_vz = trapeze_speed_filter(arm_vz, arm_z_cmd - arm_z, ARM_AMAX, ARM_DMAX, ARM_VMAX);
		arm_z += arm_vz;

		int32_t delta = ((arm_z * ARM_STEP_BY_MM) >> 16) - arm_z_step;
		if( delta > 0 )
		{
			GPIOB->ODR |= GPIO_ODR_ODR13;
			GPIOB->ODR &= ~GPIO_ODR_ODR14;
			GPIOB->ODR |= GPIO_ODR_ODR14;
			arm_z_step++;
		}
		else if( delta < 0)
		{
			GPIOB->ODR &= ~GPIO_ODR_ODR13;
			GPIOB->ODR &= ~GPIO_ODR_ODR14;
			GPIOB->ODR |= GPIO_ODR_ODR14;
			arm_z_step--;
		}

		vTaskDelay(ms_to_tick(1));
	}
}

static void arm_cmd_zab(void* arg)
{
	struct arm_cmd_zab_param* param = (struct arm_cmd_zab_param*) arg;

	arm_goto_zab(param->z, param->a, param->b);
}

int arm_goto_zab(uint32_t z, int32_t a, int32_t b)
{
	log_format(LOG_INFO, "z = %zd a = %zd b = %zd", z, a, b);

	ax12_set_goal_position(AX12_ARM_1, a);
	ax12_set_goal_position(AX12_ARM_2, b);

	// saturation pour ne pas forcer
	if( z > ARM_ZMAX )
	{
		z = ARM_ZMAX;
	}

	xSemaphoreTake(arm_mutex, portMAX_DELAY);
	arm_z_cmd = z;
	xSemaphoreGive(arm_mutex);

	return 0;
}

int arm_goto_xyz(int32_t x, int32_t y, uint32_t z)
{
	log_format(LOG_INFO, "x = %zd y = %zd z = %zd", x, y, z);

	// TODO changement repère robot vers repere articulaire
	int32_t a = 0;
	int32_t b = 0;

	arm_goto_zab(z, a, b);

	return 0;
}

void arm_bridge_on()
{
	// pwm a 50% car la pompe est en 12V
	pwm_set(PWM_BRIDGE, PWM_ARR / 2);
}

void arm_bridge_off()
{
	pwm_set(PWM_BRIDGE, 0);
}

static void arm_cmd_bridge(void* arg)
{
	uint8_t on = *((uint8_t*) arg);

	if( on )
	{
		arm_bridge_on();
	}
	else
	{
		arm_bridge_off();
	}
}