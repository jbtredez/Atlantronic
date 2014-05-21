#define WEAK_MOTION
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "kernel/log.h"
#include "kernel/can_motor.h"
#include "kernel/can/can_id.h"
#include "kernel/canopen.h"
#include "control.h"
#include "motion.h"
#include "kernel/location/location.h"
#include "kernel/geometric_model/geometric_model.h"
#include "kernel/driver/usb.h"
#include "kernel/driver/gyro.h"
#include "kernel/fault.h"

static enum motion_state motion_state;
static enum motion_status motion_status;
static struct motion_cmd_set_actuator_kinematics_arg motion_wanted_kinematics; // cinematique desiree (mode MOTION_ACTUATOR_KINEMATICS)
static Kinematics motion_kinematics[CAN_MOTOR_MAX];
static Kinematics motion_kinematics_mes[CAN_MOTOR_MAX];
static xSemaphoreHandle motion_mutex;
static float motion_ds;
static float motion_v;
static VectPlan motion_u;
static VectPlan motion_cp;
static KinematicsParameters motion_curvilinearKinematicsParam;
static Kinematics motion_curvilinearKinematics;
static VectPlan motion_cp_cmd;
static VectPlan motion_pos_mes;
static VectPlan motion_dest;  //!< destination

static void motion_compute_trajectory();
static void motion_compute_speed();

// interface usb
void motion_cmd_goto(void* arg);
void motion_cmd_set_speed(void* arg);
void motion_cmd_free(void* arg);
void motion_cmd_set_actuator_kinematics(void* arg);


static int motion_module_init()
{
	motion_mutex = xSemaphoreCreateMutex();

	if( ! motion_mutex )
	{
		return ERR_INIT_CONTROL;
	}

	usb_add_cmd(USB_CMD_MOTION_GOTO, &motion_cmd_goto);
	usb_add_cmd(USB_CMD_MOTION_SET_SPEED, &motion_cmd_set_speed);
	usb_add_cmd(USB_CMD_MOTION_FREE, &motion_cmd_free);
	usb_add_cmd(USB_CMD_MOTION_SET_ACTUATOR_KINEMATICS, &motion_cmd_set_actuator_kinematics);

	return 0;
}

module_init(motion_module_init, INIT_MOTION);

void motion_compute()
{
	xSemaphoreTake(motion_mutex, portMAX_DELAY);

	int motorNotReady = 0;
	int homing = 0;

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
			if(can_motor[2*i+1].homingStatus != CAN_MOTOR_HOMING_DONE && can_motor[2*i+1].is_op_enable())
			{
				can_motor[2*i+1].update_homing(1);
				motorNotReady = 1;
				homing = 1;
			}
		}
	}

	if( ! motorNotReady )
	{
		// mise à jour de la position
		location_update(motion_kinematics_mes, CAN_MOTOR_MAX, CONTROL_DT);
	}
	else if( motion_state != MOTION_READY_FREE)
	{
		motion_state = MOTION_READY_FREE;
		log(LOG_INFO, "all motors not ready -> MOTION_READY_FREE");
	}

	motion_pos_mes = location_get_position();

	switch(motion_state)
	{
		case MOTION_READY_FREE:
			for(int i = 0; i < CAN_MOTOR_MAX; i++)
			{
				motion_kinematics[i].pos = motion_kinematics_mes[i].pos;
				motion_kinematics[i].v = 0;
			}
			break;
		case MOTION_READY_ASSER:
			// TODO
			for(int i = 0; i < CAN_MOTOR_MAX; i++)
			{
				motion_kinematics[i].v = 0;
			}
			break;
		case MOTION_SPEED:
			motion_compute_speed();
			break;
		case MOTION_ACTUATOR_KINEMATICS:
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
			break;
		case MOTION_TRAJECTORY:
			motion_compute_trajectory();
			break;
		case MOTION_BACK_TO_WALL:
			// TODO
			break;
		default:
		case MOTION_END:
			// TODO : c'est termine, on ne bouge plus
			for(int i = 0; i < CAN_MOTOR_MAX; i++)
			{
				motion_kinematics[i].v = 0;
			}
			break;
	}

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		if( ! (homing &&  (i & 0x01)) )
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

	xSemaphoreGive(motion_mutex);
}

void motion_compute_trajectory()
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
		motion_state = MOTION_READY_ASSER;
		motion_curvilinearKinematics.v = 0;
		motion_curvilinearKinematics.a = 0;
		for(int i = 0; i < CAN_MOTOR_MAX; i++)
		{
			motion_kinematics[i].v = 0;
		}
	}
}

void motion_compute_speed()
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
}

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

void motion_cmd_free(void* arg)
{
	(void) arg;
	motion_stop(false);
}

void motion_cmd_set_actuator_kinematics(void* arg)
{
	struct motion_cmd_set_actuator_kinematics_arg* cmd = (struct motion_cmd_set_actuator_kinematics_arg*) arg;
	motion_set_actuator_kinematics(*cmd);
}

void motion_stop(bool asser)
{
	xSemaphoreTake(motion_mutex, portMAX_DELAY);
	if(motion_state != MOTION_END)
	{
		if( asser)
		{
			motion_state = MOTION_READY_ASSER;
			log(LOG_INFO, "MOTION_READY_ASSER");
		}
		else
		{
			motion_state = MOTION_READY_FREE;
			log(LOG_INFO, "MOTION_READY_FREE");
		}
	}
	xSemaphoreGive(motion_mutex);
}

void motion_goto(VectPlan dest, VectPlan cp, const KinematicsParameters &linearParam, const KinematicsParameters &angularParam)
{
	xSemaphoreTake(motion_mutex, portMAX_DELAY);

	VectPlan A = loc_to_abs(motion_pos_mes, cp);
	VectPlan B = loc_to_abs(dest, cp);
	VectPlan ab = B - A;
	float nab = ab.norm();

	if(motion_state != MOTION_READY_FREE && motion_state != MOTION_READY_ASSER)
	{
		// on fait deja quelque chose
		if( motion_state != MOTION_TRAJECTORY)
		{
			// on ne peut rien faire
			goto unlock_mutex;
		}

		// on a deja une trajectoire en cours
		if( cp.x != motion_cp.x || cp.y != motion_cp.y)
		{
			// on ne peut pas changer le cp d'un mouvement en cours
			goto unlock_mutex;
		}

		// TODO si on reste sur la meme droite, on peut changer la destination
		// pour le moment, on ne peut pas changer le mouvement en cours
		goto unlock_mutex;
	}

	motion_ds = 0;
	motion_cp = cp;

	if( nab > EPSILON)
	{
		// translation - rotation combinee
		motion_u = ab / nab;
		motion_ds = nab;
		motion_curvilinearKinematicsParam = linearParam;
		float sigmaAbs = fabsf(motion_u.theta);
		if( sigmaAbs > EPSILON)
		{
			if(motion_curvilinearKinematicsParam.vMax > angularParam.vMax / sigmaAbs)
			{
				motion_curvilinearKinematicsParam.vMax = angularParam.vMax / sigmaAbs;
			}

			if(motion_curvilinearKinematicsParam.aMax > angularParam.aMax / sigmaAbs)
			{
				motion_curvilinearKinematicsParam.aMax = angularParam.aMax / sigmaAbs;
			}

			if(motion_curvilinearKinematicsParam.dMax > angularParam.dMax / sigmaAbs)
			{
				motion_curvilinearKinematicsParam.dMax = angularParam.dMax / sigmaAbs;
			}
		}
	}
	else
	{
		// rotation pure
		motion_u.x = 0;
		motion_u.y = 0;
		motion_u.theta = ab.theta / fabsf(ab.theta);
		motion_curvilinearKinematicsParam = angularParam;
		motion_ds = fabsf(ab.theta);
	}

	if(motion_state != MOTION_END)
	{
		log_format(LOG_INFO, "goto %d %d %d", (int)dest.x, (int)dest.y, (int)(dest.theta*180/M_PI));
		motion_cp_cmd = A;
		motion_curvilinearKinematics.pos = 0;
		motion_state = MOTION_TRAJECTORY;
		motion_status = MOTION_PREPARING_MOTION;
		for(int i = 0; i < CAN_MOTOR_MAX; i++)
		{
			motion_kinematics[i] = motion_kinematics_mes[i];
		}

		motion_dest = dest;
		log(LOG_INFO, "PREPARING_MOTION");
	}

unlock_mutex:
	xSemaphoreGive(motion_mutex);
}

void motion_set_cp_speed(VectPlan cp, VectPlan u, float v)
{
	xSemaphoreTake(motion_mutex, portMAX_DELAY);

	if(motion_state != MOTION_END)
	{
		log(LOG_INFO, "set cp speed");
		motion_cp = cp;
		motion_u = u;
		motion_v = v;
		motion_state = MOTION_SPEED;
		motion_status = MOTION_PREPARING_MOTION;
		for(int i = 0; i < CAN_MOTOR_MAX; i++)
		{
			motion_kinematics[i] = motion_kinematics_mes[i];
		}

		log(LOG_INFO, "PREPARING_MOTION");
	}

	xSemaphoreGive(motion_mutex);
}

void motion_set_actuator_kinematics(struct motion_cmd_set_actuator_kinematics_arg cmd)
{
	xSemaphoreTake(motion_mutex, portMAX_DELAY);

	if(motion_state != MOTION_END)
	{
		motion_state = MOTION_ACTUATOR_KINEMATICS;
		motion_wanted_kinematics = cmd;

		for(int i = 0; i < 6; i++)
		{
			if( cmd.mode[i] != KINEMATICS_POSITION && cmd.mode[i] != KINEMATICS_SPEED)
			{
				log_format(LOG_ERROR, "unknown mode %d for actuator %i", cmd.mode[i], i);
				motion_state = MOTION_READY_FREE;
			}
		}
	}

	xSemaphoreGive(motion_mutex);
}

void motion_update_usb_data(struct control_usb_data* data)
{
	xSemaphoreTake(motion_mutex, portMAX_DELAY);

	data->motion_state = motion_state;
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
