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
#include <stdlib.h>

#define ARM_STACK_SIZE       300
#define ARM_STEP_BY_MM         5
#define ARM_HZ              1000
#define ARM_ZMAX            (200 << 16)

static xSemaphoreHandle arm_mutex;
static int32_t arm_a_cmd;     //!< commande du bras (angle a)
static int32_t arm_b_cmd;     //!< commande du bras (angle b)
static uint32_t arm_x_cmd;    //!< commande du bras (selon x)
static uint32_t arm_y_cmd;    //!< commande du bras (selon y)
static uint32_t arm_z_cmd;    //!< commande en hauteur du bras
static uint32_t arm_cmd_type; //!< type de commande

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
	.x = (110 << 16),
	.y = 0,
	.alpha = 0,
	.ca = 1 << 30,
	.sa = 0
};

static const int32_t ARM_L1 = 118 << 16;
static const int32_t ARM_L2 = 120 << 16;

//!< modèle géométrique inverse du bras
static int arm_compute_ab(int32_t x, int32_t y, int way);
static int arm_compute_xyz_loc();
static int arm_compute_xyz_abs();
static void arm_cmd_goto(void* arg);
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

	ax12_set_goal_limit(AX12_ARM_1, 0xcc, 0x332);
	ax12_set_goal_limit(AX12_ARM_2, 0x00, 0x3ff);

	usb_add_cmd(USB_CMD_ARM_GOTO, &arm_cmd_goto);
	usb_add_cmd(USB_CMD_ARM_BRIDGE, &arm_cmd_bridge);

	return 0;
}

module_init(arm_module_init, INIT_ARM);

static void arm_task()
{
	int32_t count = 0;
	// configuration des ax12
	ax12_set_moving_speed(AX12_ARM_1, 0x3ff);
	ax12_set_moving_speed(AX12_ARM_2, 0x3ff);

	ax12_set_torque_limit(AX12_ARM_1, 0x300);
	ax12_set_torque_limit(AX12_ARM_2, 0x300);

	ax12_set_torque_enable(AX12_ARM_1, 1);
	ax12_set_torque_enable(AX12_ARM_2, 1);

	// TODO procedure de mise à zéro
	ax12_set_goal_position(AX12_ARM_1, 0);
	ax12_set_goal_position(AX12_ARM_2, 0);

	arm_z = 0;
	arm_vz = 0;
	arm_z_step = 0;

	GPIOB->ODR |= GPIO_ODR_ODR12;

	while(1)
	{
		count++;
		xSemaphoreTake(arm_mutex, portMAX_DELAY);
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

		// toutes les 10ms on s'occupe des axes x et y
		if( count == 10)
		{
			count = 0;

			switch( arm_cmd_type)
			{
				default:
				case ARM_CMD_ART:
					break;
				case ARM_CMD_XYZ_ABS:
					arm_compute_xyz_abs();
					break;
				case ARM_CMD_XYZ_LOC:
					arm_compute_xyz_loc();
					break;
			}
			ax12_set_goal_position(AX12_ARM_1, arm_a_cmd);
			ax12_set_goal_position(AX12_ARM_2, arm_b_cmd);
		}

		xSemaphoreGive(arm_mutex);

		vTaskDelay(ms_to_tick(1));
	}
}

//!< way : coude haut (1) ou coude bas (-1)
static int arm_compute_ab(int32_t x, int32_t y, int way)
{
	// verification que l'objectif est dans l'espace de travail
	int64_t norm2_xy = (int64_t)x * (int64_t)x + (int64_t)y* (int64_t)y;
	int32_t norm_xy = sqrtf(norm2_xy);

	// espace accessible, x<0 => dans le robot donc interdit
	if( x < 0 || norm_xy > ARM_L1 + ARM_L2 || norm_xy < ARM_L1 - ARM_L2 )
	{
		goto error;
	}

	int64_t num = norm2_xy - (int64_t)ARM_L1 * (int64_t)ARM_L1 - (int64_t)ARM_L2 * (int64_t)ARM_L2;
	int32_t cb = (int32_t)(num / (((int64_t)ARM_L1 * (int64_t)ARM_L2) >> 15));
	int32_t b = way * fx_acos(cb);

	// prise en compte de la saturation sur b : 150 degrés => 150/360 * 2^26 = 27962026
	if( abs(b) > 27962026)
	{
		goto error;
	}

	// l1 + l2 * c2
	int32_t l1_l2c2 = ARM_L1 + (((int64_t)ARM_L2 * (int64_t)cb) >> 16);
	// l2 * s2
	int32_t l2s2 = ((int64_t)ARM_L2 * (int64_t)fx_sin(b)) >> 30;
	int32_t dx = (((int64_t)x * (int64_t)l1_l2c2) >> 20) + (((int64_t)y * (int64_t)l2s2) >> 20);
	int32_t dy = (((int64_t)y * (int64_t)l1_l2c2) >> 20) - (((int64_t)x * (int64_t)l2s2) >> 20);
	int32_t a = fx_atan2(dy, dx);

	// prise en compte de la saturation sur a : 90 degrés => 90/360 * 2^26 = 16777216
	if( abs(a) > 16777216)
	{
		goto error;
	}

	// mise a jour des commandes uniquement si c'est un mouvement réalisable
	arm_a_cmd = a;
	arm_b_cmd = b;

	return 0;

error:
	return -1;
}

static int arm_compute_xyz_loc()
{
	struct fx_vect_pos C_robot =
	{
		.x = arm_x_cmd,
		.y = arm_y_cmd,
		.alpha = 0,
		.ca = 1 << 30,
		.sa = 0
	};

	// changements de repères
	struct fx_vect_pos C_arm;
	pos_abs_to_loc(&ARM_BASE, &C_robot, &C_arm);

	int res = arm_compute_ab(C_arm.x, C_arm.y, 1);

	if(res)
	{
		return -1;
	}

	return 0;
}

static int arm_compute_xyz_abs()
{
	struct fx_vect_pos C_table =
	{
		.x = arm_x_cmd,
		.y = arm_y_cmd,
		.alpha = 0,
		.ca = 1 << 30,
		.sa = 0
	};

	// changements de repères
	struct fx_vect_pos pos_robot = location_get_position();
	struct fx_vect_pos C_robot;
	struct fx_vect_pos C_arm;
	pos_abs_to_loc(&pos_robot, &C_table, &C_robot);
	pos_abs_to_loc(&ARM_BASE, &C_robot, &C_arm);

	int res = arm_compute_ab(C_arm.x, C_arm.y, 1);

	if(res)
	{
		return -1;
	}

	return 0;
}

int arm_goto_abz(uint32_t z, int32_t a, int32_t b)
{
	log_format(LOG_INFO, "z = %d a = %d b = %d", (int)z, (int)a, (int)b);

	// saturation pour ne pas forcer
	if( z > ARM_ZMAX )
	{
		z = ARM_ZMAX;
	}

	xSemaphoreTake(arm_mutex, portMAX_DELAY);
	arm_a_cmd = a;
	arm_b_cmd = b;
	arm_z_cmd = z;
	arm_cmd_type = ARM_CMD_ART;
	xSemaphoreGive(arm_mutex);

	return 0;
}

static void arm_cmd_goto(void* arg)
{
	struct arm_cmd_goto_param* param = (struct arm_cmd_goto_param*) arg;

	switch(param->type)
	{
		case ARM_CMD_ART:
			arm_goto_abz(param->a, param->b, param->z);
			break;
		case ARM_CMD_XYZ_ABS:
		case ARM_CMD_XYZ_LOC:
			arm_goto_xyz(param->x, param->y, param->z, param->type);
			break;
		default:
			break;
	}
}

int arm_goto_xyz(int32_t x, int32_t y, uint32_t z, enum arm_cmd_type type)
{
	log_format(LOG_INFO, "x = %d y = %d z = %d", (int)x, (int)y, (int)z);

	// saturation pour ne pas forcer
	if( z > ARM_ZMAX )
	{
		z = ARM_ZMAX;
	}

	xSemaphoreTake(arm_mutex, portMAX_DELAY);
	arm_x_cmd = x;
	arm_y_cmd = y;
	arm_z_cmd = z;
	arm_cmd_type = type;
	xSemaphoreGive(arm_mutex);

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