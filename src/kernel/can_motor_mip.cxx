#include "can_motor_mip.h"
#include "kernel/module.h"
#include "kernel/log.h"
#include "kernel/robot_parameters.h"

#include <math.h>

#define ARRAY_SIZE(a)        (sizeof(a)/sizeof(a[0]))

CanMipMotor can_motor[CAN_MOTOR_MAX];

int can_motor_module_init()
{
	can_motor[CAN_MOTOR_RIGHT].nodeId = CAN_MOTOR_RIGHT_NODEID;
#if 0
	can_motor[CAN_MOTOR_RIGHT].inputGain = 60 * MOTOR_DRIVING1_RED / (float)(2 * M_PI * DRIVING1_WHEEL_RADIUS);
	can_motor[CAN_MOTOR_RIGHT].outputGain = 2 * M_PI * DRIVING1_WHEEL_RADIUS / (float)(MOTOR_ENCODER_RESOLUTION * MOTOR_DRIVING1_RED);
#endif
	can_motor[CAN_MOTOR_RIGHT].name = "moteur droit";
	can_motor[CAN_MOTOR_RIGHT].fault_disconnected_id = FAULT_CAN_MOTOR_DISCONNECTED_0;

	can_motor[CAN_MOTOR_LEFT].nodeId = CAN_MOTOR_LEFT_NODEID;
#if 0
	can_motor[CAN_MOTOR_LEFT].inputGain = 60 * MOTOR_DRIVING2_RED / (float)(2 * M_PI * DRIVING2_WHEEL_RADIUS);
	can_motor[CAN_MOTOR_LEFT].outputGain = 2 * M_PI * DRIVING2_WHEEL_RADIUS / (float)(MOTOR_DRIVING2_RED * MOTOR_ENCODER_RESOLUTION);
#endif
	can_motor[CAN_MOTOR_LEFT].name = "moteur gauche";
	can_motor[CAN_MOTOR_LEFT].fault_disconnected_id = FAULT_CAN_MOTOR_DISCONNECTED_1;

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		can_mip_register_node(&can_motor[i]);
	}

	return 0;
}

module_init(can_motor_module_init, INIT_CAN_MOTOR);

CanMipMotor::CanMipMotor()
{
	inputGain = 1;
	outputGain = 1;
/*
	kinematics.pos = 0;
	kinematics.v = 0;
	kinematics.a = 0;
	status_word = 0;
	last_status_word = 0;*/
	connected = false;
}

