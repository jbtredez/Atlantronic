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
#include "kernel/vect_pos.h"
#include "kernel/driver/usb.h"
#include "kernel/math/trigo.h"
#include "foo/location/location.h"
#include "foo/pwm.h"
#include "ax12.h"
#include <math.h>

#define ARM_STACK_SIZE       300
#define ARM_STEP_BY_MM         5
#define ARM_HZ              1000
#define ARM_ZMAX            (200 << 16)

static xSemaphoreHandle arm_mutex;
static uint32_t arm_z_cmd;    //!< commande en hauteur du bras

static int32_t arm_z;
static int32_t arm_vz;
static int32_t arm_z_step;

//!< vitesse max du moteur pas à pas
const int32_t ARM_VMAX = (125 << 16) / ARM_HZ;
//!< accélération max du moteur pas à pas
const int32_t ARM_AMAX = (400 << 16) / (ARM_HZ * ARM_HZ);
//!< décélération max du moteur pas à pas
const int32_t ARM_DMAX = (400 << 16) / (ARM_HZ * ARM_HZ);

//!< position de la base du bras dans le repère robot
const struct fx_vect_pos ARM_BASE = {
	.x = 100,
	.y = 0,
	.alpha = 0,
	.ca = 1,
	.sa = 0
};

static const int32_t ARM_L1 = 110 << 16;
static const int32_t ARM_L2 = 120 << 16;

//!< modèle géométrique inverse du bras
static int arm_compute_ab(int32_t x, int32_t y, int32_t* a, int32_t* b);
static void arm_cmd_xyz(void* arg);
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

	usb_add_cmd(USB_CMD_ARM_XYZ, &arm_cmd_xyz);
	usb_add_cmd(USB_CMD_ARM_ZAB, &arm_cmd_zab);
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

static int arm_compute_ab(int32_t x, int32_t y, int32_t* a, int32_t* b)
{
	// verification que l'objectif est dans l'espace de travail
	int64_t norm2_xy = (int64_t)x * (int64_t)x + (int64_t)y* (int64_t)y;
	int32_t norm_xy = sqrtf(norm2_xy);

	if( norm_xy > ARM_L1 + ARM_L2 || norm_xy < ARM_L1 - ARM_L2 )
	{
		log(LOG_ERROR, "espace non accessible");
		return -1;
	}

	int64_t num = norm2_xy - (int64_t)ARM_L1 * (int64_t)ARM_L1 - (int64_t)ARM_L2 * (int64_t)ARM_L2;
	int32_t cb = (int32_t)(num / ((int64_t)ARM_L1 * (int64_t)ARM_L2)) << 15;
	*b = fx_acos(cb);

	// methode 1
/*
	int32_t alpha = fx_atan2(y, x);
	num = norm2_xy + (int64_t)ARM_L1 * (int64_t)ARM_L1 - (int64_t)ARM_L2 * (int64_t)ARM_L2;
	den = 2 * ARM_L1 * norm_xy;
	cb = (int32_t)(num / den);
	int32_t beta = fx_acos(cb);

	*a = alpha - beta;
*/

	// methode 2 (voir si ça marche et si c'est plus rapide)
	num = (int64_t)(ARM_L1 + (int32_t)(((int64_t)ARM_L2 * (int64_t)cb) >> 16)) * (int64_t)x + (((int64_t)ARM_L2 * fx_sin(*b)) >> 30) * (int64_t)y;
	int32_t ca = (int32_t)(num / norm2_xy) << 16;
	*a = fx_acos(ca);

	return 0;
}

static void arm_cmd_xyz(void* arg)
{
	struct arm_cmd_xyz_param* param = (struct arm_cmd_xyz_param*) arg;

	arm_goto_xyz(param->x, param->y, param->z);
}

int arm_goto_xyz(int32_t x, int32_t y, uint32_t z)
{
	log_format(LOG_INFO, "x = %zd y = %zd z = %zd", x, y, z);

	struct fx_vect_pos C_table =
	{
		.x = x,
		.y = y,
		.alpha = 0,
		.ca = 1,
		.sa = 0
	};

	// changements de repères
	struct fx_vect_pos pos_robot = location_get_position();
	struct fx_vect_pos C_robot;
	struct fx_vect_pos C_arm;
	pos_abs_to_loc(&pos_robot, &C_table, &C_robot);
	pos_abs_to_loc(&ARM_BASE, &C_robot, &C_arm);

	int32_t a = 0;
	int32_t b = 0;

	arm_compute_ab(x, y, &a, &b);

	arm_goto_zab(z, a, b);

	return 0;
}

void arm_bridge_on()
{
	// pwm "a 50%" car la pompe est en 12V
	pwm_set(PWM_BRIDGE, (12 * PWM_ARR)/21);
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