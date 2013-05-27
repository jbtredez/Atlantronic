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
#define ARM_HZ               500
#define ARM_ZMIN        (65<<16)
#define ARM_ZMAX     (265 << 16)
#define ARM_ZINIT    (150 << 16)  //!< hauteur au lancement
#define ARM_ZMIN_ARM_CENTER      (80<<16) //!< hauteur minimale avant laquelle on doit centrer le bras

static xSemaphoreHandle arm_mutex;
static int32_t arm_a_cmd;     //!< commande du bras (angle a)
static int32_t arm_b_cmd;     //!< commande du bras (angle b)
static uint32_t arm_x_cmd;    //!< commande du bras (selon x)
static uint32_t arm_y_cmd;    //!< commande du bras (selon y)
// suppression du levage
#if 0
static uint32_t arm_z_cmd;    //!< commande en hauteur du bras
#endif
static uint32_t arm_x1_cmd;   //!< commande de la ventouse
static uint32_t arm_y1_cmd;   //!< commande de la ventouse
static uint32_t arm_x2_cmd;   //!< commande de la ventouse
static uint32_t arm_y2_cmd;   //!< commande de la ventouse
static int8_t arm_tool_way_cmd;  //!< commande du sens de la ventouse ou du crochet
static uint32_t arm_cmd_type; //!< type de commande

// suppression du levage
#if 0
static int32_t arm_z;
static int32_t arm_vz;
static int32_t arm_z_step;
#endif

//!< vitesse max du moteur pas à pas
const int32_t ARM_VMAX = (100 << 16) / ARM_HZ;
//!< accélération max du moteur pas à pas
const int32_t ARM_AMAX = (400 << 16) / (ARM_HZ * ARM_HZ);
//!< décélération max du moteur pas à pas
const int32_t ARM_DMAX = (400 << 16) / (ARM_HZ * ARM_HZ);

//!< position de la base du bras dans le repère robot
struct fx_vect_pos ARM_BASE = {
	.x = (115 << 16),
	.y = 0,
	.alpha = 0,
	.ca = 1 << 30,
	.sa = 0
};

//!< position de la ventouse par rapport au servo du bout du bras
struct fx_vect_pos VENTOUSE = {
	.x = (25 << 16),
	.y = (35 << 16),
	.alpha = (1<<23), // ventouse à 45 degrés
	.ca = 759250125,
	.sa = 759250125
};

//!< position du crochet par rapport au servo du bout du bras
struct fx_vect_pos HOOK = {
	.x = (0 << 16),
	.y = (0 << 16),
	.alpha = 0, // crochet à 0 degrés
	.ca = 1<<30,
	.sa = 0
};

//!< modèle géométrique inverse du bras
//!< way : sens du coude du bras
static int arm_compute_ab(int32_t x, int32_t y, int way);
static int arm_compute_xyz_loc();
static int arm_compute_xyz_abs();
//! ventouse_way : direction de la ventouse (-+ 45 degrés) selon la position du dernier servo
static int arm_compute_tool_abs(struct fx_vect_pos tool_pos);
static void arm_cmd_goto(void* arg);
static uint32_t arm_satz(uint32_t z);
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

	arm_tool_way_cmd = 1;

	ax12_set_goal_limit(AX12_ARM_1, 0xcc, 0x332);
	ax12_set_goal_limit(AX12_ARM_2, 0x00, 0x3ff);

	usb_add_cmd(USB_CMD_ARM_GOTO, &arm_cmd_goto);
	usb_add_cmd(USB_CMD_ARM_BRIDGE, &arm_cmd_bridge);

	return 0;
}

module_init(arm_module_init, INIT_ARM);

static void arm_task()
{
	// configuration des ax12
	ax12_auto_update(AX12_ARM_1, 1);
	ax12_auto_update(AX12_ARM_2, 1);

	ax12_set_moving_speed(AX12_ARM_1, 0x3ff);
	ax12_set_moving_speed(AX12_ARM_2, 0x3ff);

	ax12_set_torque_limit(AX12_ARM_1, 0x300);
	ax12_set_torque_limit(AX12_ARM_2, 0x300);

	ax12_set_torque_enable(AX12_ARM_1, 1);
	ax12_set_torque_enable(AX12_ARM_2, 1);

	// on va monter au début
	//arm_goto_xyz(200<<16, 0, ARM_ZINIT, ARM_CMD_XYZ_LOC);
	// rangement du bras
	arm_close();

	while(1)
	{
		xSemaphoreTake(arm_mutex, portMAX_DELAY);

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
			case ARM_CMD_VENTOUSE_ABS:
				arm_compute_tool_abs(VENTOUSE);
				break;
			case ARM_CMD_HOOK_ABS:
				arm_compute_tool_abs(HOOK);
				break;
		}

		ax12_set_goal_position(AX12_ARM_1, arm_a_cmd);
		ax12_set_goal_position(AX12_ARM_2, arm_b_cmd);

		xSemaphoreGive(arm_mutex);
		vTaskDelay(ms_to_tick(50));
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

	// choix du sens en fonction du signe de y pour prendre la solution avec le coude le plus au centre possible
	int way = 1;
	if( C_arm.y < 0)
	{
		way = -1;
	}

	int res = arm_compute_ab(C_arm.x, C_arm.y, way);

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

int arm_compute_tool_abs(struct fx_vect_pos tool_pos)
{
	struct fx_vect2 X1;
	struct fx_vect2 X2;
	struct fx_vect2 X1_robot;
	struct fx_vect2 X2_robot;
	struct fx_vect_pos pos_robot = location_get_position();

	// orientation de la doite ((x1,y1) ; (x2,y2))
	int32_t alpha = fx_atan2(arm_y2_cmd - arm_y1_cmd, arm_x2_cmd - arm_x1_cmd);

	// angle b du secondd bras dans le repere absolu
	int32_t b_abs = alpha + (1<<24) - arm_tool_way_cmd * tool_pos.alpha;
	// somme des angles a + b
	int32_t a_b = b_abs - pos_robot.alpha; 

	// translation des points X1 et X2 => point de contact en B
	int32_t cb_abs = fx_cos(b_abs);
	int32_t sb_abs = fx_sin(b_abs);
	int32_t dx = (((int64_t)(ARM_L2 + tool_pos.x) * (int64_t)cb_abs) >> 30) - arm_tool_way_cmd * (((int64_t) tool_pos.y * (int64_t)sb_abs) >> 30);
	int32_t dy = (((int64_t)(ARM_L2 + tool_pos.x) * (int64_t)sb_abs) >> 30) + arm_tool_way_cmd * (((int64_t) tool_pos.y * (int64_t)cb_abs) >> 30);
	X1.x = arm_x1_cmd - dx;
	X1.y = arm_y1_cmd - dy;
	X2.x = arm_x2_cmd - dx;
	X2.y = arm_y2_cmd - dy;

	vect2_abs_to_loc(&pos_robot, &X1, &X1_robot);
	vect2_abs_to_loc(&pos_robot, &X2, &X2_robot);
	vect2_abs_to_loc(&ARM_BASE, &X1_robot, &X1);
	vect2_abs_to_loc(&ARM_BASE, &X2_robot, &X2);

	int32_t ux = X2.x - X1.x;
	int32_t uy = X2.y - X1.y;

	// ux^2 + uy^2
	int64_t ux2_uy2 = (((int64_t) ux * (int64_t) ux) >> 16) + (((int64_t) uy * (int64_t) uy) >> 16);

	// ux * y1 - uy * x1
	int64_t uxy1_uyx1 = (int64_t) ux * (int64_t) X1.y - (int64_t) uy * (int64_t) X1.x;
	int32_t uxy1_uyx1_over_ux2_uy2 = uxy1_uyx1 / ux2_uy2;
	int64_t delta2 = ((int64_t)ARM_L1 * (int64_t)ARM_L1) / (ux2_uy2 >> 16) - ((int64_t)uxy1_uyx1_over_ux2_uy2*(int64_t)uxy1_uyx1_over_ux2_uy2);

	if(delta2 < 0)
	{
		// pas de solution
		return -1;
	}

	int32_t delta = sqrtf(delta2);
	// ux * x1 + uy * y1
	int64_t uxx1_uyy1 = (int64_t)ux * (int64_t)X1.x + (int64_t)uy * (int64_t)X1.y;
	int32_t uxx1_uyy1_over_ux2_uy2 = uxx1_uyy1 / ux2_uy2;

	// 2 solutions (ou deux fois la même dans le cas limite). On tente la premiere
	int32_t k = - uxx1_uyy1_over_ux2_uy2 + delta;
	int32_t a = fx_atan2(X1.y + (((int64_t)k * (int64_t)uy) >> 16), X1.x + ((((int64_t)k * (int64_t)ux)>>16)));

	// prise en compte de la saturation sur a : 90 degrés => 90/360 * 2^26 = 16777216
	if( abs(a) > 16777216)
	{
		// on tente la seconde solution
		k = - uxx1_uyy1_over_ux2_uy2 - delta;
		a = fx_atan2(X1.y + (((int64_t)k * (int64_t)uy) >> 16), X1.x + ((((int64_t)k * (int64_t)ux)>>16)));
		if( abs(a) > 16777216)
		{
			// pas de solutions viable
			return -1;
		}
	}

	arm_a_cmd = a;
	arm_b_cmd = a_b - a;

	return 0;
}

static uint32_t arm_satz(uint32_t z)
{
	if( z > ARM_ZMAX )
	{
		z = ARM_ZMAX;
	}
	else if( z < ARM_ZMIN)
	{
		z = ARM_ZMIN;
	}

	return z;
}

int arm_goto_abz(int32_t a, int32_t b, uint32_t z)
{
	log_format(LOG_INFO, "a = %d b = %d z = %u", (int)a, (int)b, (unsigned int)z);

	// saturation pour ne pas forcer
	z = arm_satz(z);

	xSemaphoreTake(arm_mutex, portMAX_DELAY);
	arm_a_cmd = a;
	arm_b_cmd = b;
	// suppression du levage
#if 0
	arm_z_cmd = z;
#endif
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
		case ARM_CMD_VENTOUSE_ABS:
			arm_ventouse_goto(param->x1, param->y1, param->x2, param->y2, param->z, param->tool_way);
			break;
		case ARM_CMD_HOOK_ABS:
			arm_hook_goto(param->x1, param->y1, param->x2, param->y2, param->z, param->tool_way);
			break;
		default:
			break;
	}
}

int arm_goto_xyz(int32_t x, int32_t y, uint32_t z, enum arm_cmd_type type)
{
	log_format(LOG_INFO, "x = %d y = %d z = %d", (int)x, (int)y, (int)z);

	// saturation pour ne pas forcer
	z = arm_satz(z);

	xSemaphoreTake(arm_mutex, portMAX_DELAY);
	arm_x_cmd = x;
	arm_y_cmd = y;
// suppression du levage
#if 0
	arm_z_cmd = z;
#endif
	arm_cmd_type = type;
	xSemaphoreGive(arm_mutex);

	return 0;
}

int arm_ventouse_goto(int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint32_t z, int8_t tool_way)
{
	log_format(LOG_INFO, "x1 = %d y1 = %d x2 =  %d y2 = %d z = %d", (int)x1, (int)y1, (int)x2, (int)y2, (int)z);

	// saturation pour ne pas forcer
	z = arm_satz(z);

	xSemaphoreTake(arm_mutex, portMAX_DELAY);
	arm_x1_cmd = x1;
	arm_y1_cmd = y1;
	arm_x2_cmd = x2;
	arm_y2_cmd = y2;
	// suppression du levage
#if 0
	arm_z_cmd = z;
#endif
	arm_tool_way_cmd = tool_way;
	arm_cmd_type = ARM_CMD_VENTOUSE_ABS;
	xSemaphoreGive(arm_mutex);

	return 0;
}

int arm_hook_goto(int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint32_t z, int8_t tool_way)
{
	log_format(LOG_INFO, "x1 = %d y1 = %d x2 =  %d y2 = %d z = %d", (int)x1, (int)y1, (int)x2, (int)y2, (int)z);

	// saturation pour ne pas forcer
	z = arm_satz(z);

	xSemaphoreTake(arm_mutex, portMAX_DELAY);
	arm_x1_cmd = x1;
	arm_y1_cmd = y1;
	arm_x2_cmd = x2;
	arm_y2_cmd = y2;
// suppression du levage
#if 0
	arm_z_cmd = z;
#endif
	arm_tool_way_cmd = tool_way;
	arm_cmd_type = ARM_CMD_HOOK_ABS;
	xSemaphoreGive(arm_mutex);

	return 0;
}

int arm_set_tool_way(int8_t tool_way)
{
	log_format(LOG_INFO, "servo %d", tool_way);

	xSemaphoreTake(arm_mutex, portMAX_DELAY);
	arm_tool_way_cmd = tool_way;
	xSemaphoreGive(arm_mutex);

	return 0;
}

void arm_bridge_on()
{
	// pwm "a 50%" car la pompe est en 12V
// Pompe supprimée
//	pwm_set(PWM_BRIDGE, (12 * PWM_ARR)/21);
}

void arm_bridge_off()
{
// Pompe supprimée
//	pwm_set(PWM_BRIDGE, 0);
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

void arm_close()
{
	arm_goto_abz(-10680707, 32042122, 0);
}
