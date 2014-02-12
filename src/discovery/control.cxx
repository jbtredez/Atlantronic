#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "kernel/log.h"
#include "kernel/can_motor.h"
#include "kernel/can/can_id.h"
#include "kernel/canopen.h"
#include "control.h"
#include "kernel/location/location.h"
#include "kernel/geometric_model/geometric_model.h"
#include "kernel/driver/usb.h"
#include "kernel/driver/spi.h"
#include "kernel/driver/adc.h"
#include "kernel/fault.h"

#define CONTROL_STACK_SIZE       350

VectPlan loc_pos;

static enum control_state control_state;
static enum control_status control_status;
static Kinematics control_kinematics[CAN_MOTOR_MAX];
static Kinematics control_kinematics_mes[CAN_MOTOR_MAX];
static struct control_usb_data control_usb_data;
static xSemaphoreHandle control_mutex;
static float control_ds;
static float control_v;
static VectPlan control_u;
static VectPlan control_cp;
static KinematicsParameters control_curvilinearKinematicsParam;
static Kinematics control_curvilinearKinematics;
static VectPlan control_cp_cmd;

static void control_task(void* arg);
static void control_compute();
static void control_compute_trajectory();
static void control_compute_speed();

// interface usb
void control_cmd_goto(void* arg);
void control_cmd_set_speed(void* arg);
void control_cmd_free(void* arg);
void control_cmd_set_actuator_speed(void* arg);

static int control_module_init()
{
	portBASE_TYPE err = xTaskCreate(control_task, "control", CONTROL_STACK_SIZE, NULL, PRIORITY_TASK_CONTROL, NULL);

	if(err != pdPASS)
	{
		return ERR_INIT_CONTROL;
	}

	control_mutex = xSemaphoreCreateMutex();

	if( ! control_mutex )
	{
		return ERR_INIT_CONTROL;
	}

	usb_add_cmd(USB_CMD_CONTROL_GOTO, &control_cmd_goto);
	usb_add_cmd(USB_CMD_CONTROL_SET_SPEED, &control_cmd_set_speed);
	usb_add_cmd(USB_CMD_CONTROL_FREE, &control_cmd_free);
	usb_add_cmd(USB_CMD_CONTROL_SET_ACTUATOR_SPEED, &control_cmd_set_actuator_speed);

	return 0;
}

module_init(control_module_init, INIT_CONTROL);

static void control_task(void* arg)
{
	(void) arg;
	//systime t1;
	//systime t2;
	//systime dt;
	int motorNotReady;
	int i = 0;

	uint32_t wake_time = 0;

	while(1)
	{
		int motor_update_max_abstime = wake_time + 2;
		motorNotReady = 0;

		//t1 = systick_get_time();
		canopen_update();

		for(i = 0; i < CAN_MOTOR_MAX; i++)
		{
			int res = can_motor[i].wait_update_until(motor_update_max_abstime);
			if( res )
			{
				// defaut, moteur ne repond pas
				fault((enum fault)(FAULT_CAN_MOTOR_DISCONNECTED_0 + i), FAULT_ACTIVE);
				motorNotReady = 1;

			}
			else
			{
				fault((enum fault)(FAULT_CAN_MOTOR_DISCONNECTED_0 + i), FAULT_CLEAR);
				if( ! can_motor[i].is_op_enable() )
				{
					motorNotReady = 1;
				}
			}
			control_kinematics_mes[i] = can_motor[i].kinematics;
		}

		// homing
		if( ! motorNotReady )
		{
			for(int i = 0; i < 3; i++)
			{
				if(can_motor[2*i+1].homingStatus != CAN_MOTOR_HOMING_DONE)
				{
					can_motor[2*i+1].update_homing(1);
				}
			}
		}

		xSemaphoreTake(control_mutex, portMAX_DELAY);

		if( ! motorNotReady )
		{
			// mise à jour de la position
			location_update(control_kinematics_mes, CAN_MOTOR_MAX, CONTROL_DT);
		}

		loc_pos = location_get_position();

		if( ! motorNotReady )
		{
			control_compute();
		}

		control_usb_data.current_time = systick_get_time();
		control_usb_data.control_state = control_state;
		control_usb_data.cons = control_cp_cmd;
		control_usb_data.pos = loc_pos;
		control_usb_data.pos_theta_gyro = spi_gyro_get_theta();
		control_usb_data.cons_v1 = control_kinematics[CAN_MOTOR_DRIVING1].v;
		control_usb_data.cons_v2 = control_kinematics[CAN_MOTOR_DRIVING2].v;
		control_usb_data.cons_v3 = control_kinematics[CAN_MOTOR_DRIVING3].v;
		control_usb_data.cons_theta1 = control_kinematics[CAN_MOTOR_STEERING1].pos;
		control_usb_data.cons_theta2 = control_kinematics[CAN_MOTOR_STEERING2].pos;
		control_usb_data.cons_theta3 = control_kinematics[CAN_MOTOR_STEERING3].pos;
		control_usb_data.mes_v1 = control_kinematics_mes[CAN_MOTOR_DRIVING1].v;
		control_usb_data.mes_v2 = control_kinematics_mes[CAN_MOTOR_DRIVING2].v;
		control_usb_data.mes_v3 = control_kinematics_mes[CAN_MOTOR_DRIVING3].v;
		control_usb_data.mes_theta1 = control_kinematics_mes[CAN_MOTOR_STEERING1].pos;
		control_usb_data.mes_theta2 = control_kinematics_mes[CAN_MOTOR_STEERING2].pos;
		control_usb_data.mes_theta3 = control_kinematics_mes[CAN_MOTOR_STEERING3].pos;
		control_usb_data.vBat = (float)adc_data.vBat * (float)VBAT_GAIN;
/*
		control_usb_data.control_i_right = control_an.i_right;
		control_usb_data.control_i_left = control_an.i_left;
*/
		xSemaphoreGive(control_mutex);

		usb_add(USB_CONTROL, &control_usb_data, sizeof(control_usb_data));

		//t2 = systick_get_time();
		//dt = t2 - t1;
		//log_format(LOG_INFO, "dt %d us", (int)(dt.ms*1000 + dt.ns/1000));

		vTaskDelayUntil(&wake_time, CONTROL_PERIOD);
	}
}

static void control_compute()
{
	switch(control_state)
	{
		case CONTROL_READY_FREE:
			for(int i = 0; i < CAN_MOTOR_MAX; i++)
			{
				control_kinematics[i].pos = control_kinematics_mes[i].pos;
				control_kinematics[i].v = 0;
			}
			break;
		case CONTROL_READY_ASSER:
			// TODO
			for(int i = 0; i < CAN_MOTOR_MAX; i++)
			{
				control_kinematics[i].v = 0;
			}
			break;
		case CONTROL_SPEED:
			control_compute_speed();
			break;
		case CONTROL_ACTUATOR_SPEED:
			break;
		case CONTROL_TRAJECTORY:
			control_compute_trajectory();
			break;
		case CONTROL_BACK_TO_WALL:
			// TODO
			break;
		default:
		case CONTROL_END:
			// TODO : c'est termine, on ne bouge plus
			for(int i = 0; i < CAN_MOTOR_MAX; i++)
			{
				control_kinematics[i].v = 0;
			}
			break;
	}

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		can_motor[i].set_speed(control_kinematics[i].v);
	}
}

void control_compute_trajectory()
{
	Kinematics kinematics = control_curvilinearKinematics;
	int wheelReady = 0;
	if( control_status != CONTROL_PREPARING_MOTION )
	{
		kinematics.setPosition(control_ds, 0, control_curvilinearKinematicsParam, CONTROL_DT);
	}

	VectPlan u_loc = abs_to_loc_speed(loc_pos.theta, control_u);

	float k = geometric_model_compute_actuator_cmd(control_cp, u_loc, kinematics.v, CONTROL_DT, control_kinematics, &wheelReady);
	control_curvilinearKinematics.v = k * kinematics.v;
	control_curvilinearKinematics.pos += control_curvilinearKinematics.v * CONTROL_DT;

	control_cp_cmd = control_cp_cmd + CONTROL_DT * control_curvilinearKinematics.v * control_u;

	if( control_status == CONTROL_PREPARING_MOTION && wheelReady)
	{
		log(LOG_INFO, "IN_MOTION");
		control_status = CONTROL_IN_MOTION;
	}

	if(fabsf(control_curvilinearKinematics.pos - control_ds) < EPSILON && fabsf(control_curvilinearKinematics.v) < EPSILON )
	{
		// TODO regarder en fonction des tolerances si c'est reached ou non
		log(LOG_INFO, "CONTROL_TARGET_REACHED");
		control_status = CONTROL_TARGET_REACHED;
		control_state = CONTROL_READY_ASSER;
		control_curvilinearKinematics.v = 0;
		control_curvilinearKinematics.a = 0;
		for(int i = 0; i < CAN_MOTOR_MAX; i++)
		{
			control_kinematics[i].v = 0;
		}
	}
}

void control_compute_speed()
{
	int wheelReady = 0;
	float v = 0;
	if( control_status != CONTROL_PREPARING_MOTION )
	{
		v = control_v;
	}

	geometric_model_compute_actuator_cmd(control_cp, control_u, v, CONTROL_DT, control_kinematics, &wheelReady);

	if( control_status == CONTROL_PREPARING_MOTION && wheelReady)
	{
		log(LOG_INFO, "IN_MOTION");
		control_status = CONTROL_IN_MOTION;
	}
}

void control_cmd_goto(void* arg)
{
	struct control_cmd_goto_arg* cmd = (struct control_cmd_goto_arg*) arg;
	control_goto(cmd->dest, cmd->cp, cmd->linearParam, cmd->angularParam);
}

void control_cmd_set_speed(void* arg)
{
	struct control_cmd_set_speed_arg* cmd = (struct control_cmd_set_speed_arg*) arg;
	control_set_cp_speed(cmd->cp, cmd->u, cmd->v);
}

void control_cmd_free(void* arg)
{
	(void) arg;
	control_stop(false);
}

void control_cmd_set_actuator_speed(void* arg)
{
	struct control_cmd_set_actuator_speed_arg* cmd = (struct control_cmd_set_actuator_speed_arg*) arg;
	control_set_actuator_speed(cmd->v);
}

void control_stop(bool asser)
{
	xSemaphoreTake(control_mutex, portMAX_DELAY);
	if(control_state != CONTROL_END)
	{
		if( asser)
		{
			control_state = CONTROL_READY_ASSER;
			log(LOG_INFO, "CONTROL_READY_ASSER");
		}
		else
		{
			control_state = CONTROL_READY_FREE;
			log(LOG_INFO, "CONTROL_READY_FREE");
		}
	}
	xSemaphoreGive(control_mutex);
}

void control_goto(VectPlan dest, VectPlan cp, const KinematicsParameters &linearParam, const KinematicsParameters &angularParam)
{
	xSemaphoreTake(control_mutex, portMAX_DELAY);

	VectPlan A = loc_to_abs(loc_pos, cp);
	VectPlan B = loc_to_abs(dest, cp);
	VectPlan ab = B - A;
	float nab = ab.norm();

	if(control_state != CONTROL_READY_FREE && control_state != CONTROL_READY_ASSER)
	{
		// on fait deja quelque chose
		if( control_state != CONTROL_TRAJECTORY)
		{
			// on ne peut rien faire
			goto unlock_mutex;
		}

		// on a deja une trajectoire en cours
		if( cp.x != control_cp.x || cp.y != control_cp.y)
		{
			// on ne peut pas changer le cp d'un mouvement en cours
			goto unlock_mutex;
		}

		// TODO si on reste sur la meme droite, on peut changer la destination
		// pour le moment, on ne peut pas changer le mouvement en cours
		goto unlock_mutex;
	}

	control_ds = 0;
	control_cp = cp;

	if( nab > EPSILON)
	{
		// translation - rotation combinee
		control_u = ab / nab;
		control_ds = nab;
		control_curvilinearKinematicsParam = linearParam;
		float sigmaAbs = fabsf(control_u.theta);
		if( sigmaAbs > EPSILON)
		{
			if(control_curvilinearKinematicsParam.vMax > angularParam.vMax / sigmaAbs)
			{
				control_curvilinearKinematicsParam.vMax = angularParam.vMax / sigmaAbs;
			}

			if(control_curvilinearKinematicsParam.aMax > angularParam.aMax / sigmaAbs)
			{
				control_curvilinearKinematicsParam.aMax = angularParam.aMax / sigmaAbs;
			}

			if(control_curvilinearKinematicsParam.dMax > angularParam.dMax / sigmaAbs)
			{
				control_curvilinearKinematicsParam.dMax = angularParam.dMax / sigmaAbs;
			}
		}
	}
	else
	{
		// rotation pure
		control_u.x = 0;
		control_u.y = 0;
		control_u.theta = ab.theta / fabsf(ab.theta);
		control_curvilinearKinematicsParam = angularParam;
		control_ds = fabsf(ab.theta);
	}

	if(control_state != CONTROL_END)
	{
		log_format(LOG_INFO, "goto %d %d %d", (int)dest.x, (int)dest.y, (int)(dest.theta*180/M_PI));
		control_cp_cmd = A;
		control_curvilinearKinematics.pos = 0;
		control_state = CONTROL_TRAJECTORY;
		control_status = CONTROL_PREPARING_MOTION;
		for(int i = 0; i < CAN_MOTOR_MAX; i++)
		{
			control_kinematics[i] = control_kinematics_mes[i];
		}

		control_usb_data.wanted_pos = dest;
		log(LOG_INFO, "PREPARING_MOTION");
	}

unlock_mutex:
	xSemaphoreGive(control_mutex);
}

void control_set_cp_speed(VectPlan cp, VectPlan u, float v)
{
	xSemaphoreTake(control_mutex, portMAX_DELAY);

	if(control_state != CONTROL_END)
	{
		log(LOG_INFO, "set cp speed");
		control_cp = cp;
		control_u = u;
		control_v = v;
		control_state = CONTROL_SPEED;
		control_status = CONTROL_PREPARING_MOTION;
		for(int i = 0; i < CAN_MOTOR_MAX; i++)
		{
			control_kinematics[i] = control_kinematics_mes[i];
		}

		log(LOG_INFO, "PREPARING_MOTION");
	}

	xSemaphoreGive(control_mutex);
}

void control_set_actuator_speed(float v[6])
{
	xSemaphoreTake(control_mutex, portMAX_DELAY);

	if(control_state != CONTROL_END)
	{
		log(LOG_INFO, "set actuators speed");
		control_state = CONTROL_ACTUATOR_SPEED;
		for(int i = 0; i < 6; i++)
		{
			control_kinematics[i].v = v[i];
		}
	}

	xSemaphoreGive(control_mutex);
}
