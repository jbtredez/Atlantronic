#define WEAK_MOTION
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "kernel/log.h"
#include "kernel/can/can_id.h"
#include "kernel/canopen.h"
#include "kernel/control.h"
#include "kernel/motion/motion.h"
#include "kernel/location/location.h"
#include "kernel/geometric_model/geometric_model.h"
#include "kernel/driver/usb.h"
#include "kernel/fault.h"
#include "kernel/driver/power.h"
#include "kernel/state_machine/state_machine.h"

enum
{
	MOTION_ENABLE_WANTED_UNKNOWN = -1,
	MOTION_ENABLE_WANTED_OFF = 0,
	MOTION_ENABLE_WANTED_ON = 1
};

static enum motion_status motion_status;
static enum motion_trajectory_step motion_traj_step;
static struct motion_cmd_set_actuator_kinematics_arg motion_wanted_kinematics; // cinematique desiree (mode MOTION_ACTUATOR_KINEMATICS)
static Kinematics motion_kinematics[CAN_MOTOR_MAX];
static Kinematics motion_kinematics_mes[CAN_MOTOR_MAX];
static xSemaphoreHandle motion_mutex;
static VectPlan motion_wanted_dest;
enum motion_way motion_wanted_way;
enum motion_trajectory_type motion_wanted_trajectory_type;
static KinematicsParameters motion_wanted_linearParam;
static KinematicsParameters motion_wanted_angularParam;
static float motion_ds[3];
static float motion_v;
static VectPlan motion_u;
static Kinematics motion_curvilinearKinematics;
static VectPlan motion_np_cmd;
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

static enum motion_wanted_state motion_wanted_state = MOTION_WANTED_STATE_UNKNOWN;
static void motion_state_enabled_entry();
static void motion_state_enabled_run();
static unsigned int motion_state_enabled_transition(unsigned int currentState);

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

	return 0;
}

module_init(motion_module_init, INIT_MOTION);

void motion_compute()
{
	xSemaphoreTake(motion_mutex, portMAX_DELAY);

	int motor_mes_valid = 1;

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		if( ! can_motor[i].is_op_enable() )
		{
			motor_mes_valid = 0;
		}

		motion_kinematics_mes[i] = can_motor[i].kinematics;
	}

	if( motor_mes_valid )
	{
		// mise à jour de la position
		location_update(motion_kinematics_mes, CONTROL_DT);
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
		motion_kinematics[i].mode = KINEMATICS_SPEED;
	}

	motion_update_motors();
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
		case MOTION_WANTED_STATE_ACTUATOR_KINEMATICS:
			return MOTION_ACTUATOR_KINEMATICS;
			break;
		case MOTION_WANTED_STATE_TRAJECTORY:
			return MOTION_TRAJECTORY;
			break;
		case MOTION_WANTED_STATE_SPEED:
			return MOTION_SPEED;
		case MOTION_WANTED_STATE_UNKNOWN:
		default:
			break;
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
	motion_status = MOTION_IN_MOTION;
	log(LOG_INFO, "IN_MOTION");
}

static void motion_state_speed_run()
{
	geometric_model_compute_actuator_cmd(motion_u, motion_v, CONTROL_DT, motion_kinematics);
	motion_update_motors();
}

//---------------------- Etat MOTION_TRAJECTORY -------------------------------
float motion_find_rotate(float start, float end)
{
	float dtheta = end - start;
	bool neg = dtheta < 0;

	// modulo 1 tour => retour dans [ 0 ; 2*M_PI [
	dtheta = fmodf(dtheta, 2*M_PI);
	if( neg )
	{
		dtheta += 2*M_PI;
	}

	// retour dans ] -M_PI ; M_PI ] tour
	if( dtheta > M_PI )
	{
		dtheta -= 2*M_PI;
	}

	return dtheta;
}

static float motion_compute_time(float ds, KinematicsParameters param)
{
	ds = fabsf(ds);
	float ainv = 1 / param.aMax + 1 / param.dMax;
	float v = sqrtf( 2 * ds / ainv); // vitesse si pas de palier a vMax
	if( v  < param.vMax )
	{
		// on n'a pas le temps d'atteindre vMax
		return v * ainv;
	}

	return ds / param.vMax + 0.5f * param.vMax * ainv;
}

static void motion_state_trajectory_entry()
{
	float dtheta1 = 0;
	float ds = 0;
	float dtheta2 = 0;
	float t = 0;

	if( motion_wanted_trajectory_type == MOTION_AXIS_A)
	{
		motion_wanted_dest.x = motion_pos_mes.x;
		motion_wanted_dest.y = motion_pos_mes.y;
	}

	VectPlan ab = motion_wanted_dest - motion_pos_mes;
	float nab = ab.norm();

	// distance minimale de translation TODO 5mm en dur
	if( nab > 5 )
	{
		float theta1 = atan2f(ab.y, ab.x);
		ds = nab;
		motion_u = ab / nab;
		motion_u.theta = 0;

		if(motion_wanted_way == WAY_FORWARD)
		{
			dtheta1 = motion_find_rotate(motion_pos_mes.theta, theta1);
			if( motion_wanted_trajectory_type == MOTION_AXIS_XYA )
			{
				dtheta2 = motion_find_rotate(motion_pos_mes.theta + dtheta1, motion_wanted_dest.theta);
			}
		}
		else if( motion_wanted_way == WAY_BACKWARD)
		{
			dtheta1 = motion_find_rotate(motion_pos_mes.theta, theta1 + M_PI);
			if( motion_wanted_trajectory_type == MOTION_AXIS_XYA )
			{
				dtheta2 = motion_find_rotate(motion_pos_mes.theta + dtheta1, motion_wanted_dest.theta);
			}
		}
		else
		{
			float dtheta1_forward = motion_find_rotate(motion_pos_mes.theta, theta1);
			float dtheta1_backward = motion_find_rotate(motion_pos_mes.theta, theta1 + M_PI);
			float dtheta2_forward = 0;
			float dtheta2_backward = 0;

			if( motion_wanted_trajectory_type == MOTION_AXIS_XYA )
			{
				dtheta2_forward = motion_find_rotate(motion_pos_mes.theta + dtheta1_forward, motion_wanted_dest.theta);
				dtheta2_backward = motion_find_rotate(motion_pos_mes.theta + dtheta1_backward, motion_wanted_dest.theta);
			}

			if ( fabsf(dtheta1_forward) + fabsf(dtheta2_forward) > fabsf(dtheta1_backward) + fabsf(dtheta2_backward))
			{
				dtheta1 = dtheta1_backward;
				dtheta2 = dtheta2_backward;
			}
			else
			{
				dtheta1 = dtheta1_forward;
				dtheta2 = dtheta2_forward;
			}
		}
	}
	else if( motion_wanted_trajectory_type != MOTION_AXIS_XY)
	{
		dtheta2 = motion_find_rotate(motion_pos_mes.theta, motion_wanted_dest.theta);
	}

	if( motion_wanted_trajectory_type == MOTION_AXIS_XY )
	{
		motion_wanted_dest.theta = motion_pos_mes.theta + dtheta1;
	}

	t = motion_compute_time(dtheta1, motion_wanted_angularParam);
	t += motion_compute_time(ds, motion_wanted_linearParam);
	t += motion_compute_time(dtheta2, motion_wanted_angularParam);

	log_format(LOG_INFO, "goto %d %d %d t %d ms", (int)motion_wanted_dest.x, (int)motion_wanted_dest.y, (int)(motion_wanted_dest.theta*180/M_PI), (int)(1000*t));
	log_format(LOG_INFO, "rotate %d translate %d, rotate %d", (int)(dtheta1 * 180 / M_PI), (int)ds, (int)(dtheta2 * 180 / M_PI));

	motion_ds[0] = dtheta1;
	motion_ds[1] = ds;
	motion_ds[2] = dtheta2;

	motion_curvilinearKinematics.reset();
	motion_traj_step = MOTION_TRAJECTORY_PRE_ROTATE;
	log(LOG_INFO, "IN_MOTION");
	motion_status = MOTION_IN_MOTION;
	motion_np_cmd = motion_pos_mes;

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		motion_kinematics[i] = motion_kinematics_mes[i];
	}

	motion_dest = motion_wanted_dest;
}

static void motion_state_trajectory_run()
{
	Kinematics kinematics = motion_curvilinearKinematics;
	KinematicsParameters curvilinearKinematicsParam;
	VectPlan u;
	float ds = 0;

	ds = motion_ds[motion_traj_step];
	if( motion_traj_step == MOTION_TRAJECTORY_PRE_ROTATE || motion_traj_step == MOTION_TRAJECTORY_ROTATE )
	{
		u = VectPlan(0, 0, 1);
		curvilinearKinematicsParam = motion_wanted_angularParam;
	}
	else
	{
		u = motion_u;
		curvilinearKinematicsParam = motion_wanted_linearParam;
	}

	kinematics.setPosition(ds, 0, curvilinearKinematicsParam, CONTROL_DT);
	VectPlan u_loc = abs_to_loc_speed(motion_pos_mes.theta, u);
	float k = geometric_model_compute_actuator_cmd(u_loc, kinematics.v, CONTROL_DT, motion_kinematics);
	motion_curvilinearKinematics.v = k * kinematics.v;
	motion_curvilinearKinematics.pos += motion_curvilinearKinematics.v * CONTROL_DT;

	motion_np_cmd = motion_np_cmd + CONTROL_DT * motion_curvilinearKinematics.v * u;

	if(fabsf(motion_curvilinearKinematics.pos - ds) < EPSILON && fabsf(motion_curvilinearKinematics.v) < EPSILON )
	{
		for(int i = 0; i < CAN_MOTOR_MAX; i++)
		{
			motion_kinematics[i].v = 0;
		}

		if( motion_traj_step == MOTION_TRAJECTORY_PRE_ROTATE)
		{
			log_format(LOG_DEBUG1, "ds %d pos %d", (int)(1000*ds), (int)(1000*motion_curvilinearKinematics.pos));
			motion_curvilinearKinematics.reset();
			motion_traj_step = MOTION_TRAJECTORY_STRAIGHT;
		}
		else if( motion_traj_step == MOTION_TRAJECTORY_STRAIGHT)
		{
			motion_curvilinearKinematics.reset();
			motion_traj_step = MOTION_TRAJECTORY_ROTATE;
		}
		else
		{
			// TODO regarder en fonction des tolerances si c'est reached ou non
			motion_curvilinearKinematics.v = 0;
			motion_curvilinearKinematics.a = 0;

			log(LOG_INFO, "MOTION_TARGET_REACHED");
			motion_status = MOTION_TARGET_REACHED;
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
	motion_goto(cmd->dest, cmd->cp, (enum motion_way)cmd->way, (enum motion_trajectory_type)cmd->type, cmd->linearParam, cmd->angularParam);
}

void motion_cmd_set_speed(void* arg)
{
	struct motion_cmd_set_speed_arg* cmd = (struct motion_cmd_set_speed_arg*) arg;
	motion_set_speed(cmd->u, cmd->v);
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
	can_motor[0].set_max_current(maxCurrent);
	can_motor[1].set_max_current(maxCurrent);
}

void motion_goto(VectPlan dest, VectPlan cp, enum motion_way way, enum motion_trajectory_type type, const KinematicsParameters &linearParam, const KinematicsParameters &angularParam)
{
	xSemaphoreTake(motion_mutex, portMAX_DELAY);
	motion_wanted_state = MOTION_WANTED_STATE_TRAJECTORY;
	motion_wanted_dest = loc_to_abs(dest, -cp);
	motion_wanted_trajectory_type = type;
	motion_wanted_way = way;
	motion_wanted_linearParam = linearParam;
	motion_wanted_angularParam = angularParam;
	xSemaphoreGive(motion_mutex);
}

void motion_set_speed(VectPlan u, float v)
{
	xSemaphoreTake(motion_mutex, portMAX_DELAY);

	motion_wanted_state = MOTION_WANTED_STATE_SPEED;
	motion_u = u;
	motion_v = v;
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		motion_kinematics[i] = motion_kinematics_mes[i];
	}

	xSemaphoreGive(motion_mutex);
}

void motion_stop()
{
	motion_set_speed(VectPlan(1,0,0), 0);
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

void motion_get_state(enum motion_state* state, enum motion_status* status, enum motion_trajectory_step* step, enum motion_wanted_state* wanted_state)
{
	xSemaphoreTake(motion_mutex, portMAX_DELAY);
	*state = (enum motion_state)motionStateMachine.getCurrentState();
	*status = motion_status;
	*step = motion_traj_step;
	*wanted_state = motion_wanted_state;
	xSemaphoreGive(motion_mutex);
}

void motion_update_usb_data(struct control_usb_data* data)
{
	xSemaphoreTake(motion_mutex, portMAX_DELAY);

	data->motion_state = motionStateMachine.getCurrentState();
	data->cons = motion_np_cmd;

	data->wanted_pos = motion_dest;

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		data->cons_motors_v[i] = motion_kinematics[i].v;
		data->mes_motors[i] = motion_kinematics_mes[i];
		data->mes_motor_current[i] = can_motor[i].current;
	}

	xSemaphoreGive(motion_mutex);
}
