#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include "kernel/log.h"
#include "kernel/can_motor.h"
#include "kernel/can/can_id.h"
#include "kernel/canopen.h"
#include "control.h"
#include "kernel/math/vect_plan.h"
#include "kernel/location/odometry.h"

#define CONTROL_STACK_SIZE       350

VectPlan Turret[3] =
{
	VectPlan(   0,  155, 0),
	VectPlan(   0, -155, 0),
	VectPlan(-175,    0, 0)
};

VectPlan loc_pos; // TODO

static void control_task(void* arg);
static void control_compute();

static int control_module_init()
{
	portBASE_TYPE err = xTaskCreate(control_task, "control", CONTROL_STACK_SIZE, NULL, PRIORITY_TASK_CONTROL, NULL);

	if(err != pdPASS)
	{
		return ERR_INIT_CONTROL;
	}

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

		if( ! res )
		{
			// mise à jour de la position
			//location_update();
			// TODO mettre odometrie au bon endroit
			float phi1 = can_motor[CAN_MOTOR_STEERING1].position;
			float phi2 = can_motor[CAN_MOTOR_STEERING2].position;
			float phi3 = can_motor[CAN_MOTOR_STEERING3].position;

			VectPlan v[3];
			v[0].x = can_motor[CAN_MOTOR_DRIVING1].speed * cos(phi1);
			v[0].y = can_motor[CAN_MOTOR_DRIVING1].speed * sin(phi1);
			v[1].x = can_motor[CAN_MOTOR_DRIVING2].speed * cos(phi2);
			v[1].y = can_motor[CAN_MOTOR_DRIVING2].speed * sin(phi2);
			v[2].x = can_motor[CAN_MOTOR_DRIVING3].speed * cos(phi3);
			v[2].y = can_motor[CAN_MOTOR_DRIVING3].speed * sin(phi3);

			VectPlan cp;
			float slippageSpeed = 0;
			VectPlan cpSpeed = odometry2turret(cp, Turret[0], Turret[1], v[0], v[1], &slippageSpeed);
			loc_pos = loc_pos + 0.005f * cpSpeed;
			log_format(LOG_INFO, "pos %d %d %d speed %d %d %d v %d %d %d phi %d %d %d", (int)loc_pos.x, (int)loc_pos.y, (int)(loc_pos.theta*180/M_PI),
					(int)cpSpeed.x, (int)cpSpeed.y, (int)(cpSpeed.theta*180/M_PI),
					(int)can_motor[CAN_MOTOR_DRIVING1].speed, (int)can_motor[CAN_MOTOR_DRIVING2].speed, (int)can_motor[CAN_MOTOR_DRIVING3].speed,
					(int)(phi1 * 180 / M_PI), (int)(phi2 * 180 / M_PI), (int)(phi3 * 180 / M_PI));

			// recuperation des entrées AN
			//adc_get(&control_an);

			control_compute();

			//t2 = systick_get_time();
			//systime dt = t2 - t1;
			//log_format(LOG_INFO, "dt %d us", (int)(dt.ms*1000 + dt.ns/1000));
		}

wait:
		vTaskDelayUntil(&wake_time, CONTROL_PERIOD);
	}
}

Kinematics control_kinematics[6];
KinematicsParameters paramDriving = {1000, 500, 500};
KinematicsParameters paramSteering = {1, 1, 1};

static void control_compute()
{
	// test
	VectPlan speedCmd(0,0,0);
	VectPlan vOnTurret[3];

	VectPlan cp(0,0,0);
	vOnTurret[0] = transferSpeed(cp, Turret[0], speedCmd);
	vOnTurret[1] = transferSpeed(cp, Turret[1], speedCmd);
	vOnTurret[2] = transferSpeed(cp, Turret[2], speedCmd);

	float v[3];
	v[0] = vOnTurret[0].norm();
	v[1] = vOnTurret[1].norm();
	v[2] = vOnTurret[2].norm();

	float w[3];
	w[0] = atan2f(vOnTurret[0].y, vOnTurret[0].x);
	w[1] = atan2f(vOnTurret[1].y, vOnTurret[1].x);
	w[2] = atan2f(vOnTurret[2].y, vOnTurret[2].x);

	control_kinematics[0].setSpeed(v[0], paramDriving, 0.005f);
	control_kinematics[1].setSpeed(v[1], paramDriving, 0.005f);
	control_kinematics[2].setSpeed(v[2], paramDriving, 0.005f);
	control_kinematics[3].setPosition(w[0], 0, paramSteering, 0.005f);
	control_kinematics[4].setPosition(w[1], 0, paramSteering, 0.005f);
	control_kinematics[5].setPosition(w[2], 0, paramSteering, 0.005f);

	//log_format(LOG_INFO, "pos = %d v = %d", (int)can_motor[CAN_MOTOR_DRIVING1].position, (int)can_motor[CAN_MOTOR_DRIVING1].speed);
	//log_format(LOG_INFO, "v %5d %5d %5d w %6d %6d %6d", (int)v[0], (int)v[1], (int)v[2], (int)(w[0]*180/3.1415f), (int)(w[1]*180/3.1415f), (int)(w[2]*180/3.1415f));
	can_motor[CAN_MOTOR_DRIVING1].set_speed(control_kinematics[0].v);
	can_motor[CAN_MOTOR_DRIVING2].set_speed(control_kinematics[1].v);
	can_motor[CAN_MOTOR_DRIVING3].set_speed(control_kinematics[2].v);
	// TODO calculer vitesse depuis position

	can_motor[CAN_MOTOR_STEERING1].set_speed(control_kinematics[3].v);
	can_motor[CAN_MOTOR_STEERING2].set_speed(control_kinematics[4].v);
	can_motor[CAN_MOTOR_STEERING3].set_speed(control_kinematics[5].v);
}
