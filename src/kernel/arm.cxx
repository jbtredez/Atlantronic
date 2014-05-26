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
#include "kernel/state_machine/state_machine.h"
#include "kernel/driver/power.h"
#include <math.h>
#include <stdlib.h>

#define ARM_STACK_SIZE       300

//static struct arm_cmd arm_pos_wanted; //!< position desiree du bras
static struct arm_cmd arm_pos_cmd; //!< position commandee du bras
static struct arm_cmd arm_pos_mes;
static xSemaphoreHandle arm_mutex;
static int arm_cmd_type;
static void arm_disabled_run();
static unsigned int arm_disabled_transition(unsigned int currentState);

static void arm_homing_run();
static unsigned int arm_homing_transition(unsigned int currentState);

static MatrixHomogeneous arm_transform;
static StateMachineState arm_states[ARM_STATE_MAX] = {
		{ "ARM_STATE_DISABLED", &nop_function, &arm_disabled_run, &arm_disabled_transition},
		{ "ARM_STATE_HOMING", &nop_function, &arm_homing_run, &arm_homing_transition},
};
static StateMachine arm_stateMachine(arm_states, ARM_STATE_MAX);

static void arm_task(void* arg);
static void arm_update_position();
static void arm_cmd(void* arg);

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

	usb_add_cmd(USB_CMD_ARM, &arm_cmd);

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

		arm_stateMachine.execute();

		xSemaphoreGive(arm_mutex);
		vTaskDelay(ms_to_tick(50));
	}
}

static void arm_update_position()
{
	bool allDynamixelsOk = 1;
	struct dynamixel_error error;
	arm_pos_mes.val[ARM_AXIS_SHOULDER] = ax12.get_position(AX12_ARM_SHOULDER, &error);
	if( error.transmit_error )
	{
		allDynamixelsOk = 0;
	}
	arm_pos_mes.val[ARM_AXIS_SHOULDER_ELBOW] = ax12.get_position(AX12_ARM_SHOULDER_ELBOW, &error);
	if( error.transmit_error )
	{
		allDynamixelsOk = 0;
	}
	arm_pos_mes.val[ARM_AXIS_WRIST_ELBOW] = ax12.get_position(AX12_ARM_WRIST_ELBOW, &error);
	if( error.transmit_error )
	{
		allDynamixelsOk = 0;
	}
	arm_pos_mes.val[ARM_AXIS_WRIST] = ax12.get_position(AX12_ARM_WRIST, &error);
	if( error.transmit_error )
	{
		allDynamixelsOk = 0;
	}
	arm_pos_mes.val[ARM_AXIS_SLIDER] = rx24.get_position(RX24_ARM_SLIDER, &error);

	if( allDynamixelsOk )
	{
		arm_transform.setIdentity();

		float theta2 = asinf( (ARM_SLIDER_POSITION_Y - ARM_SLIDER_L1 * sinf(arm_pos_mes.val[ARM_AXIS_SLIDER])) / ARM_SLIDER_L2 );
		float z0 = ARM_SLIDER_POSITION_Z + ARM_SLIDER_L1 * cosf(arm_pos_mes.val[ARM_AXIS_SLIDER]) + ARM_SLIDER_L2 * cosf(theta2);
		arm_transform.translate(ARM_SHOULDER_POSITION_X, 0, z0);
		arm_transform.rotateY( arm_pos_mes.val[ARM_AXIS_SHOULDER] );
		arm_transform.translate(ARM_DIST_SHOULDER_TO_SHOULDER_ELBOW, 0, 0);
		arm_transform.rotateZ( -arm_pos_mes.val[ARM_AXIS_SHOULDER_ELBOW] );
		arm_transform.translate(ARM_DIST_SHOULDER_ELBOW_TO_WRIST_ELBOW, 0, 0);
		arm_transform.rotateY( -arm_pos_mes.val[ARM_AXIS_WRIST_ELBOW] );
		arm_transform.translate(ARM_DIST_WRIST_ELBOW_TO_WRIST, 0, 0);
		arm_transform.rotateZ( arm_pos_mes.val[ARM_AXIS_WRIST] );
		arm_transform.translate(ARM_DIST_WRIST_TO_SUCKER, 0, 0);
	}
}
/*
void arm_goto(struct arm_cmd cmd)
{
	xSemaphoreTake(arm_mutex, portMAX_DELAY);
	arm_pos_wanted = cmd;
	xSemaphoreGive(arm_mutex);
}
*/
void arm_get_matrix(float* mat)
{
	xSemaphoreTake(arm_mutex, portMAX_DELAY);
	memcpy(mat, arm_transform.val, sizeof(arm_transform.val));
	xSemaphoreGive(arm_mutex);
}

void arm_update()
{
	// TODO verif pas de depassement en hauteur
	ax12.set_goal_position(AX12_ARM_SHOULDER, arm_pos_cmd.val[ARM_AXIS_SHOULDER]);
	ax12.set_goal_position(AX12_ARM_SHOULDER_ELBOW, arm_pos_cmd.val[ARM_AXIS_SHOULDER_ELBOW]);
	ax12.set_goal_position(AX12_ARM_WRIST_ELBOW, arm_pos_cmd.val[ARM_AXIS_WRIST_ELBOW]);
	ax12.set_goal_position(AX12_ARM_WRIST, arm_pos_cmd.val[ARM_AXIS_WRIST]);
	rx24.set_goal_position(RX24_ARM_SLIDER, arm_pos_cmd.val[ARM_AXIS_SLIDER]);
}

//---------------------- Etat ARM_STATE_DISABLED ------------------------------
static void arm_disabled_run()
{

}

static unsigned int arm_disabled_transition(unsigned int currentState)
{
	if( ! power_get() && arm_cmd_type == ARM_CMD_HOMING )
	{
		return ARM_STATE_HOMING;
	}

	return currentState;
}

//---------------------- Etat ARM_STATE_HOMING --------------------------------
static void arm_homing_run()
{
	// TODO mesurer la position homing + voir si on bouge les servos 1 par 1
	arm_pos_cmd.val[ARM_AXIS_SHOULDER] = 0;
	arm_pos_cmd.val[ARM_AXIS_SHOULDER_ELBOW] = 0;
	arm_pos_cmd.val[ARM_AXIS_WRIST_ELBOW] = 0;
	arm_pos_cmd.val[ARM_AXIS_WRIST] = 0;

	arm_update();
}

static unsigned int arm_homing_transition(unsigned int currentState)
{
	if( power_get() || arm_cmd_type == ARM_CMD_DISABLE )
	{
		return ARM_STATE_DISABLED;
	}

	return currentState;
}

//---------------------- Fin StateMachine -------------------------------------

static void arm_cmd(void* arg)
{
	struct arm_cmd* cmd_arg = (struct arm_cmd*) arg;
	arm_cmd_type = cmd_arg->cmdType;
}
