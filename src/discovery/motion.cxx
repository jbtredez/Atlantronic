#define WEAK_MOTION
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "kernel/log.h"
#include "kernel/can/can_id.h"
#include "kernel/canopen.h"
#include "control.h"
#include "motion.h"
#include "kernel/location/location.h"
#include "kernel/geometric_model/geometric_model.h"
#include "kernel/driver/usb.h"
#include "kernel/driver/gyro.h"
#include "kernel/fault.h"
#include "kernel/driver/power.h"
#include "kernel/state_machine/state_machine.h"

enum
{
	MOTION_ENABLE_WANTED_UNKNOWN = -1,
	MOTION_ENABLE_WANTED_OFF = 0,
	MOTION_ENABLE_WANTED_ON = 1
};

enum
{
	MOTION_WANTED_STATE_UNKNOWN = 0,
	MOTION_WANTED_STATE_HOMING,
	MOTION_WANTED_STATE_ACTUATOR_KINEMATICS,
	MOTION_WANTED_STATE_SPEED,
	MOTION_WANTED_STATE_TRAJECTORY,
};

static enum motion_status motion_status;
static struct motion_cmd_set_actuator_kinematics_arg motion_wanted_kinematics; // cinematique desiree (mode MOTION_ACTUATOR_KINEMATICS)
static Kinematics motion_kinematics[CAN_MOTOR_MAX];
static Kinematics motion_kinematics_mes[CAN_MOTOR_MAX];
static xSemaphoreHandle motion_mutex;
static VectPlan motion_wanted_dest;
static VectPlan motion_wanted_cp;
static KinematicsParameters motion_wanted_linearParam;
static KinematicsParameters motion_wanted_angularParam;
static float motion_ds;
static float motion_v;
static VectPlan motion_u;
static VectPlan motion_cp;
static KinematicsParameters motion_curvilinearKinematicsParam;
static Kinematics motion_curvilinearKinematics;
static VectPlan motion_cp_cmd;
static VectPlan motion_pos_mes;
static VectPlan motion_dest;  //!< destination

static void motion_update_motors();

// machine a etats de motion
static uint8_t motion_enable_wanted = MOTION_ENABLE_WANTED_UNKNOWN;
static void motion_state_disabled_entry();
static void motion_state_disabled_run();
static unsigned int motion_state_disabled_transition(unsigned int currentState);

static void motion_state_try_enabled_run();
static unsigned int motion_state_try_enable_transition(unsigned int currentState);

static uint8_t motion_wanted_state = MOTION_WANTED_STATE_UNKNOWN;
static void motion_state_enabled_entry();
static void motion_state_enabled_run();
static unsigned int motion_state_enabled_transition(unsigned int currentState);

static void motion_state_homing_run();
static unsigned int motion_state_homing_transition(unsigned int currentState);

static void motion_state_actuator_kinematics_run();

static void motion_state_speed_run();
static void motion_state_speed_entry();

static void motion_state_trajectory_entry();
static void motion_state_trajectory_run();
static unsigned int motion_state_trajectory_transition(unsigned int currentState);

static unsigned int motion_state_generic_power_transition(unsigned int currentState);

StateMachineState motionStates[MOTION_MAX_STATE] = {
		{ "MOTION_DISABLED", &motion_state_disabled_entry, &motion_state_disabled_run, &motion_state_disabled_transition},
		{ "MOTION_TRY_ENABLE", &nop_function, &motion_state_try_enabled_run, &motion_state_try_enable_transition },
		{ "MOTION_ENABLED", &motion_state_enabled_entry, &motion_state_enabled_run, &motion_state_enabled_transition},
		{ "MOTION_HOMING", &nop_function, &motion_state_homing_run, &motion_state_homing_transition},
		{ "MOTION_ACTUATOR_KINEMATICS", &nop_function, &motion_state_actuator_kinematics_run, &motion_state_generic_power_transition},
		{ "MOTION_SPEED", &motion_state_speed_entry, &motion_state_speed_run, &motion_state_generic_power_transition},
		{ "MOTION_TRAJECTORY", &motion_state_trajectory_entry, &motion_state_trajectory_run, &motion_state_trajectory_transition},
};
StateMachine motionStateMachine(motionStates, MOTION_MAX_STATE);

// interface usb
void motion_cmd_goto(void* arg);
void motion_cmd_set_speed(void* arg);
void motion_cmd_enable(void* arg);
void motion_cmd_set_actuator_kinematics(void* arg);
void motion_cmd_homing(void* arg);
void motion_cmd_set_max_current(void* arg);

static int motion_module_init()
{
	motion_mutex = xSemaphoreCreateMutex();

	if( ! motion_mutex )
	{
		return ERR_INIT_CONTROL;
	}

	usb_add_cmd(USB_CMD_MOTION_GOTO, &motion_cmd_goto);
	usb_add_cmd(USB_CMD_MOTION_SET_SPEED, &motion_cmd_set_speed);
	usb_add_cmd(USB_CMD_MOTION_SET_MAX_CURRENT, &motion_cmd_set_max_current);
	usb_add_cmd(USB_CMD_MOTION_ENABLE, &motion_cmd_enable);
	usb_add_cmd(USB_CMD_MOTION_SET_ACTUATOR_KINEMATICS, &motion_cmd_set_actuator_kinematics);
	usb_add_cmd(USB_CMD_MOTION_HOMING, &motion_cmd_homing);

	return 0;
}

module_init(motion_module_init, INIT_MOTION);

void motion_compute()
{
	xSemaphoreTake(motion_mutex, portMAX_DELAY);

	int motorNotReady = 0;

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		if( ! can_motor[i].is_op_enable() )
		{
			motorNotReady = 1;
		}

		motion_kinematics_mes[i] = can_motor[i].kinematics;
	}

	// homing
	if( ! motorNotReady )
	{
		for(int i = 0; i < 3; i++)
		{
			if( can_motor[2*i+1].homingStatus != CAN_MOTOR_HOMING_DONE )
			{
				motorNotReady = 1;
			}
		}
	}

	if( ! motorNotReady )
	{
		// mise à jour de la position
		location_update(motion_kinematics_mes, CAN_MOTOR_MAX, CONTROL_DT);
	}

	motion_pos_mes = location_get_position();

	motionStateMachine.execute();

	xSemaphoreGive(motion_mutex);
}

static void motion_update_motors()
{
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		if( motion_kinematics[i].mode == KINEMATICS_SPEED )
		{
			can_motor[i].set_speed(motion_kinematics[i].v);
		}
		else if( motion_kinematics[i].mode == KINEMATICS_POSITION )
		{
			can_motor[i].set_position(motion_kinematics[i].pos);
		}
	}
}

//---------------------- Etat MOTION_DISABLED ---------------------------------
static void motion_state_disabled_entry()
{
	motion_enable_wanted = MOTION_ENABLE_WANTED_UNKNOWN;
}

static void motion_state_disabled_run()
{
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		motion_kinematics[i].pos = motion_kinematics_mes[i].pos;
		motion_kinematics[i].v = 0;
		can_motor[i].enable(false);
	}
}

static unsigned int motion_state_disabled_transition(unsigned int currentState)
{
	if( power_get() )
	{
		// puissance desactivee
		motion_enable_wanted = MOTION_ENABLE_WANTED_UNKNOWN;
	}

	if( motion_enable_wanted == MOTION_ENABLE_WANTED_ON )
	{
		return MOTION_TRY_ENABLE;
	}

	return currentState;
}

//---------------------- Etat MOTION_TRY_ENABLE -------------------------------
static void motion_state_try_enabled_run()
{
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		motion_kinematics[i].pos = motion_kinematics_mes[i].pos;
		motion_kinematics[i].v = 0;
		can_motor[i].enable(true);
	}
}

static unsigned int motion_state_try_enable_transition(unsigned int currentState)
{
	bool all_op_enable = true;

	if( power_get() || motion_enable_wanted == MOTION_ENABLE_WANTED_OFF )
	{
		// puissance desactivee
		return MOTION_DISABLED;
	}

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		all_op_enable &= can_motor[i].is_op_enable();
	}

	if( all_op_enable )
	{
		// tout les moteurs sont en op_enable
		return MOTION_ENABLED;
	}

	return currentState;
}

//---------------------- Etat MOTION_ENABLED ----------------------------------
static void motion_state_enabled_entry()
{
	if( motion_enable_wanted == MOTION_ENABLE_WANTED_ON )
	{
		motion_enable_wanted = MOTION_ENABLE_WANTED_UNKNOWN;
	}
	motion_wanted_state = MOTION_WANTED_STATE_UNKNOWN;
}

static void motion_state_enabled_run()
{
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		motion_kinematics[i].v = 0;
	}
}

static unsigned int motion_state_enabled_transition(unsigned int currentState)
{
	bool all_op_enable = true;

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		all_op_enable &= can_motor[i].is_op_enable();
	}

	if( power_get() || ! all_op_enable || motion_enable_wanted == MOTION_ENABLE_WANTED_OFF )
	{
		return MOTION_DISABLED;
	}

	switch(motion_wanted_state)
	{
		case MOTION_WANTED_STATE_HOMING:
			return MOTION_HOMING;
			break;
		case MOTION_WANTED_STATE_ACTUATOR_KINEMATICS:
			return MOTION_ACTUATOR_KINEMATICS;
			break;
		case MOTION_WANTED_STATE_TRAJECTORY:
			return MOTION_TRAJECTORY;
			break;
		default:
			break;
	}

	return currentState;
}

//---------------------- Etat MOTION_HOMING -----------------------------------
static void motion_state_homing_run()
{
	for(int i = 0; i < 3; i++)
	{
		if(can_motor[2*i+1].homingStatus != CAN_MOTOR_HOMING_DONE && can_motor[2*i+1].is_op_enable())
		{
			can_motor[2*i+1].update_homing(1);
		}
	}
}

static unsigned int motion_state_homing_transition(unsigned int currentState)
{
	bool all_op_enable = true;
	bool homingDone = true;

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		all_op_enable &= can_motor[i].is_op_enable();
	}

	if( power_get() || ! all_op_enable || motion_enable_wanted == MOTION_ENABLE_WANTED_OFF)
	{
		return MOTION_DISABLED;
	}

	for(int i = 0; i < 3; i++)
	{
		if( can_motor[2*i+1].homingStatus != CAN_MOTOR_HOMING_DONE )
		{
			can_motor[2*i+1].update_homing(1);
			homingDone = false;
		}
	}

	if( homingDone )
	{
		return MOTION_ENABLED;
	}

	return currentState;
}

//---------------------- Etat MOTION_ACTUATOR_KINEMATICS ----------------------
static void motion_state_actuator_kinematics_run()
{
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		if(motion_wanted_kinematics.mode[i] == KINEMATICS_SPEED)
		{
			motion_kinematics[i].v = motion_wanted_kinematics.val[i];
			motion_kinematics[i].mode = KINEMATICS_SPEED;
		}
		else if(motion_wanted_kinematics.mode[i] == KINEMATICS_POSITION)
		{
			motion_kinematics[i].pos = motion_wanted_kinematics.val[i];
			motion_kinematics[i].mode = KINEMATICS_POSITION;
		}
	}

	motion_update_motors();
}

//---------------------- Etat MOTION_SPEED ------------------------------------
static void motion_state_speed_entry()
{
	motion_status = MOTION_PREPARING_MOTION;
	log(LOG_INFO, "PREPARING_MOTION");
}

static void motion_state_speed_run()
{
	int wheelReady = 0;
	float v = 0;
	if( motion_status != MOTION_PREPARING_MOTION )
	{
		v = motion_v;
	}

	geometric_model_compute_actuator_cmd(motion_cp, motion_u, v, CONTROL_DT, motion_kinematics, &wheelReady);

	if( motion_status == MOTION_PREPARING_MOTION && wheelReady)
	{
		log(LOG_INFO, "IN_MOTION");
		motion_status = MOTION_IN_MOTION;
	}

	motion_update_motors();
}

//---------------------- Etat MOTION_TRAJECTORY -------------------------------
static void motion_state_trajectory_entry()
{
	VectPlan A = loc_to_abs(motion_pos_mes, motion_wanted_cp);
	VectPlan B = loc_to_abs(motion_wanted_dest, motion_wanted_cp);
	VectPlan ab = B - A;
	float nab = ab.norm();

	motion_ds = 0;
	motion_cp = motion_wanted_cp;

	if( nab > EPSILON)
	{
		// translation - rotation combinee
		motion_u = ab / nab;
		motion_ds = nab;
		motion_curvilinearKinematicsParam = motion_wanted_linearParam;
		float sigmaAbs = fabsf(motion_u.theta);
		if( sigmaAbs > EPSILON)
		{
			if(motion_curvilinearKinematicsParam.vMax > motion_wanted_angularParam.vMax / sigmaAbs)
			{
				motion_curvilinearKinematicsParam.vMax = motion_wanted_angularParam.vMax / sigmaAbs;
			}

			if(motion_curvilinearKinematicsParam.aMax > motion_wanted_angularParam.aMax / sigmaAbs)
			{
				motion_curvilinearKinematicsParam.aMax = motion_wanted_angularParam.aMax / sigmaAbs;
			}

			if(motion_curvilinearKinematicsParam.dMax > motion_wanted_angularParam.dMax / sigmaAbs)
			{
				motion_curvilinearKinematicsParam.dMax = motion_wanted_angularParam.dMax / sigmaAbs;
			}
		}
	}
	else
	{
		// rotation pure
		motion_u.x = 0;
		motion_u.y = 0;
		motion_u.theta = ab.theta / fabsf(ab.theta);
		motion_curvilinearKinematicsParam = motion_wanted_angularParam;
		motion_ds = fabsf(ab.theta);
	}

	log_format(LOG_INFO, "goto %d %d %d", (int)motion_wanted_dest.x, (int)motion_wanted_dest.y, (int)(motion_wanted_dest.theta*180/M_PI));
	motion_cp_cmd = A;
	motion_curvilinearKinematics.pos = 0;
	motion_status = MOTION_PREPARING_MOTION;
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		motion_kinematics[i] = motion_kinematics_mes[i];
	}

	motion_dest = motion_wanted_dest;
	log(LOG_INFO, "PREPARING_MOTION");
}

static void motion_state_trajectory_run()
{
	Kinematics kinematics = motion_curvilinearKinematics;
	int wheelReady = 0;
	if( motion_status != MOTION_PREPARING_MOTION )
	{
		kinematics.setPosition(motion_ds, 0, motion_curvilinearKinematicsParam, CONTROL_DT);
	}

	VectPlan u_loc = abs_to_loc_speed(motion_pos_mes.theta, motion_u);

	float k = geometric_model_compute_actuator_cmd(motion_cp, u_loc, kinematics.v, CONTROL_DT, motion_kinematics, &wheelReady);
	motion_curvilinearKinematics.v = k * kinematics.v;
	motion_curvilinearKinematics.pos += motion_curvilinearKinematics.v * CONTROL_DT;

	motion_cp_cmd = motion_cp_cmd + CONTROL_DT * motion_curvilinearKinematics.v * motion_u;

	if( motion_status == MOTION_PREPARING_MOTION && wheelReady)
	{
		log(LOG_INFO, "IN_MOTION");
		motion_status = MOTION_IN_MOTION;
	}

	if(fabsf(motion_curvilinearKinematics.pos - motion_ds) < EPSILON && fabsf(motion_curvilinearKinematics.v) < EPSILON )
	{
		// TODO regarder en fonction des tolerances si c'est reached ou non
		log(LOG_INFO, "MOTION_TARGET_REACHED");
		motion_status = MOTION_TARGET_REACHED;
		motion_curvilinearKinematics.v = 0;
		motion_curvilinearKinematics.a = 0;
		for(int i = 0; i < CAN_MOTOR_MAX; i++)
		{
			motion_kinematics[i].v = 0;
		}
	}

	motion_update_motors();
}

static unsigned int motion_state_trajectory_transition(unsigned int currentState)
{
	unsigned int newState = motion_state_generic_power_transition(currentState);
	if( newState != currentState )
	{
		return newState;
	}

	if( motion_status == MOTION_TARGET_REACHED )
	{
		return MOTION_ENABLED;
	}

	return currentState;
}

//---------------------- Transitions generiques -------------------------------
static unsigned int motion_state_generic_power_transition(unsigned int currentState)
{
	bool all_op_enable = true;

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		all_op_enable &= can_motor[i].is_op_enable();
	}

	if( power_get() || ! all_op_enable || motion_enable_wanted == MOTION_ENABLE_WANTED_OFF)
	{
		return MOTION_DISABLED;
	}

	if( motion_enable_wanted == MOTION_ENABLE_WANTED_ON )
	{
		return MOTION_ENABLED;
	}

	return currentState;
}

//---------------------- fin state machine ------------------------------------

void motion_cmd_goto(void* arg)
{
	struct motion_cmd_goto_arg* cmd = (struct motion_cmd_goto_arg*) arg;
	motion_goto(cmd->dest, cmd->cp, cmd->linearParam, cmd->angularParam);
}

void motion_cmd_set_speed(void* arg)
{
	struct motion_cmd_set_speed_arg* cmd = (struct motion_cmd_set_speed_arg*) arg;
	motion_set_cp_speed(cmd->cp, cmd->u, cmd->v);
}

void motion_cmd_set_max_current(void* arg)
{
	struct motion_cmd_set_max_driving_current_arg* cmd = (struct motion_cmd_set_max_driving_current_arg*) arg;
	motion_set_max_driving_current(cmd->maxDrivingCurrent);
}

void motion_cmd_enable(void* arg)
{
	struct motion_cmd_enable_arg* cmd_arg = (struct motion_cmd_enable_arg*) arg;

	motion_enable(cmd_arg->enable != 0);
}

void motion_cmd_set_actuator_kinematics(void* arg)
{
	struct motion_cmd_set_actuator_kinematics_arg* cmd = (struct motion_cmd_set_actuator_kinematics_arg*) arg;
	motion_set_actuator_kinematics(*cmd);
}

void motion_cmd_homing(void* arg)
{
	(void) arg;
	motion_homing();
}

void motion_homing()
{
	xSemaphoreTake(motion_mutex, portMAX_DELAY);
	motion_wanted_state = MOTION_WANTED_STATE_HOMING;
	xSemaphoreGive(motion_mutex);
}

void motion_enable(bool enable)
{
	xSemaphoreTake(motion_mutex, portMAX_DELAY);
	if( enable )
	{
		motion_enable_wanted = MOTION_ENABLE_WANTED_ON;
	}
	else
	{
		motion_enable_wanted = MOTION_ENABLE_WANTED_OFF;
	}
	xSemaphoreGive(motion_mutex);
}

void motion_set_max_driving_current(float maxCurrent)
{
	can_motor[CAN_MOTOR_DRIVING1].set_max_current(maxCurrent);
	can_motor[CAN_MOTOR_DRIVING2].set_max_current(maxCurrent);
	can_motor[CAN_MOTOR_DRIVING3].set_max_current(maxCurrent);
}

void motion_goto(VectPlan dest, VectPlan cp, const KinematicsParameters &linearParam, const KinematicsParameters &angularParam)
{
	xSemaphoreTake(motion_mutex, portMAX_DELAY);
	motion_wanted_state = MOTION_WANTED_STATE_TRAJECTORY;
	motion_wanted_dest = dest;
	motion_wanted_cp = cp;
	motion_wanted_linearParam = linearParam;
	motion_wanted_angularParam = angularParam;
	xSemaphoreGive(motion_mutex);
}

void motion_set_cp_speed(VectPlan cp, VectPlan u, float v)
{
	xSemaphoreTake(motion_mutex, portMAX_DELAY);

	motion_wanted_state = MOTION_WANTED_STATE_SPEED;
	motion_cp = cp;
	motion_u = u;
	motion_v = v;
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		motion_kinematics[i] = motion_kinematics_mes[i];
	}

	xSemaphoreGive(motion_mutex);
}

void motion_set_actuator_kinematics(struct motion_cmd_set_actuator_kinematics_arg cmd)
{
	xSemaphoreTake(motion_mutex, portMAX_DELAY);

	motion_wanted_state = MOTION_WANTED_STATE_ACTUATOR_KINEMATICS;
	motion_wanted_kinematics = cmd;

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		if( cmd.mode[i] != KINEMATICS_POSITION && cmd.mode[i] != KINEMATICS_SPEED)
		{
			log_format(LOG_ERROR, "unknown mode %d for actuator %i", cmd.mode[i], i);
			motion_wanted_state = MOTION_WANTED_STATE_UNKNOWN;
		}
	}

	xSemaphoreGive(motion_mutex);
}

void motion_update_usb_data(struct control_usb_data* data)
{
	xSemaphoreTake(motion_mutex, portMAX_DELAY);

	data->motion_state = motionStateMachine.getCurrentState();
	data->cons = motion_cp_cmd;
	data->cons_v1 = motion_kinematics[CAN_MOTOR_DRIVING1].v;
	data->cons_v2 = motion_kinematics[CAN_MOTOR_DRIVING2].v;
	data->cons_v3 = motion_kinematics[CAN_MOTOR_DRIVING3].v;
	data->cons_theta1 = motion_kinematics[CAN_MOTOR_STEERING1].pos;
	data->cons_theta2 = motion_kinematics[CAN_MOTOR_STEERING2].pos;
	data->cons_theta3 = motion_kinematics[CAN_MOTOR_STEERING3].pos;
	data->wanted_pos = motion_dest;
	data->mes_v1 = motion_kinematics_mes[CAN_MOTOR_DRIVING1].v;
	data->mes_v2 = motion_kinematics_mes[CAN_MOTOR_DRIVING2].v;
	data->mes_v3 = motion_kinematics_mes[CAN_MOTOR_DRIVING3].v;
	data->mes_theta1 = motion_kinematics_mes[CAN_MOTOR_STEERING1].pos;
	data->mes_theta2 = motion_kinematics_mes[CAN_MOTOR_STEERING2].pos;
	data->mes_theta3 = motion_kinematics_mes[CAN_MOTOR_STEERING3].pos;

	xSemaphoreGive(motion_mutex);
}
