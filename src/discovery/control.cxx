#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "kernel/log.h"
#include "kernel/can_motor.h"
#include "kernel/can/can_id.h"
#include "kernel/canopen.h"
#include "control.h"
#include "kernel/location/odometry.h"
#include "kernel/driver/usb.h"

#define CONTROL_STACK_SIZE       350

VectPlan Turret[3] =
{
	VectPlan(   0,  155, 0),
	VectPlan(   0, -155, 0),
	VectPlan(-175,    0, 0)
};

VectPlan loc_pos; // TODO
VectPlan loc_npSpeed; // TODO

static enum control_state control_state;
static enum control_status control_status;
static Kinematics control_kinematics[6];
static KinematicsParameters paramDriving = {2000, 2000, 2000}; // TODO
static KinematicsParameters paramSteering = {2, 2, 2}; // TODO
static struct control_usb_data control_usb_data;
static xSemaphoreHandle control_mutex;
static float control_ds;
static VectPlan control_u;
static VectPlan control_cp;
static KinematicsParameters control_curvilinearKinematicsParam;
static Kinematics control_curvilinearKinematics;
static VectPlan control_cp_cmd;

static void control_task(void* arg);
static void control_compute();
static void control_compute_speed(VectPlan cp, VectPlan u, float speed);
static void control_compute_trajectory();

// interface usb
void control_cmd_goto(void* arg);

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

	usb_add_cmd(USB_CMD_CONTROL_SET_TRAJECTORY, &control_cmd_goto);

	return 0;
}

module_init(control_module_init, INIT_CONTROL);

static void control_task(void* arg)
{
	(void) arg;
	//systime t1;
	//systime t2;
	int res;

	uint32_t wake_time = 0;

	while(1)
	{
		//log(LOG_INFO, "sync");
		//t1 = systick_get_time();

		if((can_motor[CAN_MOTOR_DRIVING1].status_word & 0x6f) != 0x27)
		{
			goto wait;
		}

		can_motor[CAN_MOTOR_DRIVING1].wait_update(0);
		canopen_sync();
		res = can_motor[CAN_MOTOR_DRIVING1].wait_update(ms_to_tick(2));

		xSemaphoreTake(control_mutex, portMAX_DELAY);

		if( ! res )
		{
			// mise à jour de la position
			//location_update();
			// TODO mettre odometrie au bon endroit
			float phi1 = can_motor[CAN_MOTOR_STEERING1].position;
			float phi2 = can_motor[CAN_MOTOR_STEERING2].position;
			float phi3 = can_motor[CAN_MOTOR_STEERING3].position;

			//log_format(LOG_INFO, "%d %d %d", (int)(phi1 * 180 / M_PI), (int)(phi2 * 180 / M_PI), (int)(phi3 * 180 / M_PI));
			VectPlan v[3];
			v[0].x = can_motor[CAN_MOTOR_DRIVING1].speed * cosf(phi1);
			v[0].y = can_motor[CAN_MOTOR_DRIVING1].speed * sinf(phi1);
			v[1].x = can_motor[CAN_MOTOR_DRIVING2].speed * cosf(phi2);
			v[1].y = can_motor[CAN_MOTOR_DRIVING2].speed * sinf(phi2);
			v[2].x = can_motor[CAN_MOTOR_DRIVING3].speed * cosf(phi3);
			v[2].y = can_motor[CAN_MOTOR_DRIVING3].speed * sinf(phi3);

			float slippageSpeed = 0;
			loc_npSpeed = odometry2turret(VectPlan(0,0,0), Turret[0], Turret[1], v[0], v[1], &slippageSpeed);
			loc_pos = loc_pos + CONTROL_DT * loc_to_abs_speed(loc_pos.theta, loc_npSpeed);

			// recuperation des entrées AN
			//adc_get(&control_an);

			control_compute();

			//t2 = systick_get_time();
			//systime dt = t2 - t1;
			//log_format(LOG_INFO, "dt %d us", (int)(dt.ms*1000 + dt.ns/1000));
		}

wait:
		control_usb_data.current_time = systick_get_time();
		control_usb_data.control_state = control_state;
		control_usb_data.cons_x = control_cp_cmd.x;
		control_usb_data.cons_y = control_cp_cmd.y;
		control_usb_data.cons_theta = control_cp_cmd.theta;
		control_usb_data.pos_x = loc_pos.x;
		control_usb_data.pos_y = loc_pos.y;
		control_usb_data.pos_theta = loc_pos.theta;
		control_usb_data.cons_v1 = control_kinematics[0].v;
		control_usb_data.cons_v2 = control_kinematics[1].v;
		control_usb_data.cons_v3 = control_kinematics[2].v;
		control_usb_data.cons_theta1 = control_kinematics[3].pos;
		control_usb_data.cons_theta2 = control_kinematics[4].pos;
		control_usb_data.cons_theta3 = control_kinematics[5].pos;
/*
		control_usb_data.control_i_right = control_an.i_right;
		control_usb_data.control_i_left = control_an.i_left;
*/
		xSemaphoreGive(control_mutex);

		usb_add(USB_CONTROL, &control_usb_data, sizeof(control_usb_data));

		vTaskDelayUntil(&wake_time, CONTROL_PERIOD);
	}
}

static void control_compute()
{
	switch(control_state)
	{
		case CONTROL_READY_FREE:
			// TODO
			break;
		case CONTROL_READY_ASSER:
			// TODO
			break;
		case CONTROL_SPEED:
			//control_compute_speed();
			// TODO
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
			break;
	}
}

void control_compute_speed(VectPlan cp, VectPlan u, float speed)
{
	for(int i = 0; i < 3; i++)
	{
		VectPlan vOnTurret = transferSpeed(cp, Turret[i], u);
		float n2 = vOnTurret.norm2();
		float v = speed * sqrtf(n2);
		float theta = atan2f(vOnTurret.y, vOnTurret.x);
		float theta_old = control_kinematics[i+3].pos;

		// on minimise la rotation des roues
		float dtheta1 = fmodf(theta - theta_old, 2*M_PI);
		if( dtheta1 > M_PI)
		{
			dtheta1 -= 2*M_PI;
		}
		else if(dtheta1 < -M_PI)
		{
			dtheta1 += 2*M_PI;
		}

		float dtheta2 = fmodf(theta + M_PI - theta_old, 2*M_PI);
		if( dtheta2 > M_PI)
		{
			dtheta2 -= 2*M_PI;
		}
		else if(dtheta2 < -M_PI)
		{
			dtheta2 += 2*M_PI;
		}

		if( fabsf(dtheta1) < fabsf(dtheta2) )
		{
			theta = theta_old + dtheta1;
		}
		else
		{
			theta = theta_old + dtheta2;
			v *= -1;
		}

		control_kinematics[i].setSpeed(v, paramDriving, CONTROL_DT);
		float w = - u.theta * speed * (u.x * vOnTurret.x + vOnTurret.y * u.y) / n2;
		control_kinematics[i+3].setPosition(theta, w, paramSteering, CONTROL_DT);
	}

	can_motor[CAN_MOTOR_DRIVING1].set_speed(control_kinematics[0].v);
	can_motor[CAN_MOTOR_DRIVING2].set_speed(control_kinematics[1].v);
	can_motor[CAN_MOTOR_DRIVING3].set_speed(control_kinematics[2].v);

	can_motor[CAN_MOTOR_STEERING1].set_speed(control_kinematics[3].v);
	can_motor[CAN_MOTOR_STEERING2].set_speed(control_kinematics[4].v);
	can_motor[CAN_MOTOR_STEERING3].set_speed(control_kinematics[5].v);
}

void control_compute_trajectory()
{
	if( control_status != CONTROL_PREPARING_MOTION )
	{
		control_curvilinearKinematics.setPosition(control_ds, 0, control_curvilinearKinematicsParam, CONTROL_DT);
	}

	control_cp_cmd = control_cp_cmd + CONTROL_DT * control_curvilinearKinematics.v * control_u;

	VectPlan u_loc = abs_to_loc_speed(loc_pos.theta, control_u);
//log_format(LOG_INFO, "%d %d", (int)(180/M_PI * (control_cp_cmd.theta - loc_pos.theta)), (int)(1000*(control_curvilinearKinematics.v - loc_npSpeed.norm())));
	control_compute_speed(control_cp, u_loc, control_curvilinearKinematics.v);

	if( control_status == CONTROL_PREPARING_MOTION )
	{
		if( fabsf(control_kinematics[3].v) < EPSILON &&
			fabsf(control_kinematics[4].v) < EPSILON &&
			fabsf(control_kinematics[5].v) < EPSILON )
		{
			control_status = CONTROL_IN_MOTION;
		}
	}
}

void control_cmd_goto(void* arg)
{
	struct control_cmd_goto_arg* cmd = (struct control_cmd_goto_arg*) arg;
	VectPlan dest(cmd->dest_x, cmd->dest_y, cmd->dest_theta);
	VectPlan cp(cmd->cp_x, cmd->cp_y, cmd->cp_theta);
	KinematicsParameters linearParam;
	linearParam.vMax = cmd->linearParam_vMax;
	linearParam.dMax = cmd->linearParam_dMax;
	linearParam.aMax = cmd->linearParam_aMax;

	KinematicsParameters angularParam;
	angularParam.vMax = cmd->angularParam_vMax;
	angularParam.dMax = cmd->angularParam_dMax;
	angularParam.aMax = cmd->angularParam_aMax;

	control_goto(dest, cp, linearParam, angularParam);
}

void control_goto(VectPlan dest, VectPlan cp, const KinematicsParameters &linearParam, const KinematicsParameters &angularParam)
{
	xSemaphoreTake(control_mutex, portMAX_DELAY);

	control_cp_cmd = loc_to_abs(loc_pos, cp);
	VectPlan ab = loc_to_abs(dest, cp) - control_cp_cmd;
	float nab = ab.norm();
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
		control_curvilinearKinematics.pos = 0;
		control_state = CONTROL_TRAJECTORY;
		control_status = CONTROL_PREPARING_MOTION;
	}
	xSemaphoreGive(control_mutex);
}
