#include "kernel/module.h"
#include "gate.h"
#include "kernel/driver/hokuyo.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/kinematics_model/KinematicsModelDiff.h"
#include "kernel/driver/encoder/EncoderSimulFromKinematicsModel.h"
#include "kernel/driver/encoder/EncoderAB.h"
#include "robot_parameters.h"
#include "kernel/control.h"

KinematicsParameters paramDriving = {1800, 1500, 1500};
KinematicsParameters linearParam = {1000, 1500, 1500};
KinematicsParameters angularParam = {3, 5, 5};

Hokuyo hokuyo[HOKUYO_MAX];
Dynamixel parasol;
DynamixelManager ax12;
//DynamixelManager rx24;

Location location;
Detection detection;
KinematicsModelDiff odoWheelKinematicsModelDiff(GATE_VOIE_ODO, paramDriving);
KinematicsModelDiff motorKinematicsModelDiff(GATE_VOIE_MOT, paramDriving);
Motion motion;
Trajectory trajectory;
PwmMotor motionMotors[MOTION_MOTOR_MAX];
EncoderSimulFromKinematicsModel motionMotorEncoder[MOTION_MOTOR_MAX];
EncoderAB motionEncoders[MOTION_MOTOR_MAX];

static int gate_robot_module_init()
{
	ax12.init("ax12", UART5_HALF_DUPLEX, 200000, AX12_MAX_ID, DYNAMIXEL_TYPE_AX12);
	//rx24.init("rx24", UART4_FULL_DUPLEX, 200000, RX24_MAX_ID, DYNAMIXEL_TYPE_RX24);

	parasol.init(&ax12, AX12_GATE_PARASOL);

	hokuyo[0].init(USART3_FULL_DUPLEX, "hokuyo1", HOKUYO1, &location);
	hokuyo[0].setPosition(VectPlan( 0, 0, 0), 1);
	hokuyo[0].scan.theta_min = -M_PI;
	hokuyo[0].scan.theta_max = M_PI;
	hokuyo[0].scan.min_object_size = 1;
	hokuyo[0].scan.min_distance = 125;
/*
	hokuyo[1].init(USART1_FULL_DUPLEX, "hokuyo2", HOKUYO2, &location);
	hokuyo[1].setPosition(VectPlan(0, 0, 0), 1);
	hokuyo[1].scan.theta_min = -M_PI;
	hokuyo[1].scan.theta_max = M_PI;
	hokuyo[1].scan.min_object_size = 1;
	hokuyo[1].scan.min_distance = 100;
*/
	location.init(&odoWheelKinematicsModelDiff);
	detection.init(&hokuyo[0], &hokuyo[1], &location);

	motionMotorEncoder[MOTION_MOTOR_LEFT].init(&location, &motorKinematicsModelDiff, LEFT_WHEEL);
	motionMotorEncoder[MOTION_MOTOR_RIGHT].init(&location, &motorKinematicsModelDiff, RIGHT_WHEEL);
	motionEncoders[MOTION_MOTOR_LEFT].init(ENCODER_1, GATE_ODO1_WAY * 2 * M_PI * GATE_ODO1_WHEEL_RADIUS / (float)(GATE_ODO_ENCODER_RESOLUTION));
	motionEncoders[MOTION_MOTOR_RIGHT].init(ENCODER_2, GATE_ODO2_WAY * 2 * M_PI * GATE_ODO2_WHEEL_RADIUS / (float)(GATE_ODO_ENCODER_RESOLUTION));

	motionMotors[MOTION_MOTOR_LEFT].name = "moteur gauche";
	motionMotors[MOTION_MOTOR_LEFT].pwmId = PWM_1;
	motionMotors[MOTION_MOTOR_LEFT].inputGain = 60 * GATE_MOTOR_DRIVING1_RED / (float)(2 * M_PI * GATE_DRIVING1_WHEEL_RADIUS) * GATE_MOTOR_RPM_TO_VOLT;
	motionMotors[MOTION_MOTOR_LEFT].encoder = &motionMotorEncoder[MOTION_MOTOR_LEFT];
	motionMotors[MOTION_MOTOR_LEFT].pid.init(3, 0, 0, 1000);

	motionMotors[MOTION_MOTOR_RIGHT].name = "moteur droit";
	motionMotors[MOTION_MOTOR_RIGHT].pwmId = PWM_2;
	motionMotors[MOTION_MOTOR_RIGHT].inputGain = 60 * GATE_MOTOR_DRIVING2_RED / (float)(2 * M_PI * GATE_DRIVING2_WHEEL_RADIUS) * GATE_MOTOR_RPM_TO_VOLT;
	motionMotors[MOTION_MOTOR_RIGHT].encoder = &motionMotorEncoder[MOTION_MOTOR_RIGHT];
	motionMotors[MOTION_MOTOR_RIGHT].pid.init(3, 0, 0, 1000);





	motion.init(&detection, &location, &motorKinematicsModelDiff, &motionMotors[MOTION_MOTOR_LEFT], &motionMotors[MOTION_MOTOR_RIGHT], &motionEncoders[MOTION_MOTOR_LEFT], &motionEncoders[MOTION_MOTOR_RIGHT]);
	trajectory.init(&detection, &motion, &location, linearParam, angularParam);

	return 0;
}

module_init(gate_robot_module_init, INIT_MAIN_ROBOT);
