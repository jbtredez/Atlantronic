//! @file arm.c
//! @brief Gestion du bras
//! @author Atlantronic

#define WEAK_ARM
#include "arm.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "kernel/systick.h"
#include "kernel/rcc.h"
#include "kernel/log.h"
#include "kernel/driver/usb.h"
#include "kernel/driver/dynamixel.h"
#include <math.h>
#include <stdlib.h>

#define ARM_STACK_SIZE       300

static struct arm_cmd arm_pos_wanted; //!< position desiree du bras
static struct arm_cmd arm_pos_cmd; //!< position commandee du bras
static struct arm_cmd arm_pos_mes;

static xSemaphoreHandle arm_mutex;
static MatrixHomogeneous arm_transform;

static void arm_task(void* arg);
static void arm_update_position();

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

	// TODO
	ax12.set_goal_limit(AX12_ARM_SHOULDER, -M_PI_2, M_PI_2);
	ax12.set_goal_limit(AX12_ARM_SHOULDER_ELBOW, -M_PI_2, M_PI_2);
	ax12.set_goal_limit(AX12_ARM_WRIST_ELBOW, -M_PI_2, M_PI_2);
	ax12.set_goal_limit(AX12_ARM_WRIST, -M_PI_2, M_PI_2);
	rx24.set_goal_limit(RX24_ARM_SLIDER, -M_PI_2, M_PI_2);

	return 0;
}

module_init(arm_module_init, INIT_ARM);

static void arm_task(void* arg)
{
	(void) arg;
/*
	// configuration des ax12
	ax12_set_moving_speed(AX12_ARM_1, 0x3ff);
	ax12_set_moving_speed(AX12_ARM_2, 0x3ff);

	ax12_set_torque_limit(AX12_ARM_1, 0x300);
	ax12_set_torque_limit(AX12_ARM_2, 0x300);

	ax12_set_torque_enable(AX12_ARM_1, 1);
	ax12_set_torque_enable(AX12_ARM_2, 1);
*/

	while(1)
	{
		xSemaphoreTake(arm_mutex, portMAX_DELAY);

		arm_update_position();

		switch( arm_pos_wanted.cmdType)
		{
			case ARM_CMD_ART:
				arm_pos_cmd = arm_pos_wanted;
				break;
			default:
				break;
		}

		// TODO verif pas de depassement en hauteur
		/*ax12.set_goal_position(AX12_ARM_SHOULDER, arm_pos_cmd.val[ARM_AXIS_SHOULDER]);
		ax12.set_goal_position(AX12_ARM_SHOULDER_ELBOW, arm_pos_cmd.val[ARM_AXIS_SHOULDER_ELBOW]);
		ax12.set_goal_position(AX12_ARM_WRIST_ELBOW, arm_pos_cmd.val[ARM_AXIS_WRIST_ELBOW]);
		ax12.set_goal_position(AX12_ARM_WRIST, arm_pos_cmd.val[ARM_AXIS_WRIST]);
		rx24.set_goal_position(RX24_ARM_SLIDER, arm_pos_cmd.val[ARM_AXIS_SLIDER]);
*/
		xSemaphoreGive(arm_mutex);
		vTaskDelay(ms_to_tick(50));
	}
}

static void arm_update_position()
{
	// TODO regarder l erreur
	struct dynamixel_error error;
	arm_pos_mes.val[ARM_AXIS_SHOULDER] = ax12.get_position(AX12_ARM_SHOULDER, &error);
	arm_pos_mes.val[ARM_AXIS_SHOULDER_ELBOW] = ax12.get_position(AX12_ARM_SHOULDER_ELBOW, &error);
	arm_pos_mes.val[ARM_AXIS_WRIST_ELBOW] = ax12.get_position(AX12_ARM_WRIST_ELBOW, &error);
	arm_pos_mes.val[ARM_AXIS_WRIST] = ax12.get_position(AX12_ARM_WRIST, &error);
	arm_pos_mes.val[ARM_AXIS_SLIDER] = rx24.get_position(RX24_ARM_SLIDER, &error);

	arm_transform.setIdentity();

	// TODO calculer z0
	arm_transform.translate(ARM_SHOULDER_POSITION_X, 0, 0);
	arm_transform.rotateY( arm_pos_mes.val[ARM_AXIS_SHOULDER] );
	arm_transform.translate(ARM_DIST_SHOULDER_TO_SHOULDER_ELBOW, 0, 0);
	arm_transform.rotateZ( -arm_pos_mes.val[ARM_AXIS_SHOULDER_ELBOW] );
	arm_transform.translate(ARM_DIST_SHOULDER_ELBOW_TO_WRIST_ELBOW, 0, 0);
	arm_transform.rotateY( -arm_pos_mes.val[ARM_AXIS_WRIST_ELBOW] );
	arm_transform.translate(ARM_DIST_WRIST_ELBOW_TO_WRIST, 0, 0);
	arm_transform.rotateZ( arm_pos_mes.val[ARM_AXIS_WRIST] );
	arm_transform.translate(ARM_DIST_WRIST_TO_SUCKER, 0, 0);
}

void arm_goto(struct arm_cmd cmd)
{
	xSemaphoreTake(arm_mutex, portMAX_DELAY);
	arm_pos_wanted = cmd;
	xSemaphoreGive(arm_mutex);
}

void arm_get_matrix(float* mat)
{
	xSemaphoreTake(arm_mutex, portMAX_DELAY);
	memcpy(mat, arm_transform.val, sizeof(arm_transform.val));
	xSemaphoreGive(arm_mutex);
}
