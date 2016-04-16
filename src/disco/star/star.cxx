#include "kernel/module.h"
#include "star.h"
#include "kernel/driver/hokuyo.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/kinematics_model/KinematicsModelDiff.h"
#include "kernel/driver/encoder/EncoderAB.h"
#include "robot_parameters.h"
#include "kernel/log.h"

KinematicsParameters paramDriving = {1800, 10000, 10000};
KinematicsParameters linearParam = {1200, 6000, 6000};
KinematicsParameters angularParam = {3, 5, 5};

Hokuyo hokuyo[HOKUYO_MAX];
Dynamixel leftWing;
Dynamixel rightWing;
Dynamixel leftCarpet;
Dynamixel rightCarpet;
Dynamixel lowFinger;
Dynamixel highFinger;
Dynamixel rightFinger;
Dynamixel leftFinger;
DynamixelManager ax12;
//DynamixelManager rx24;

Location location;
Detection detection;
KinematicsModelDiff odoWheelKinematicsModelDiff(STAR_VOIE_ODO, paramDriving);
KinematicsModelDiff motorKinematicsModelDiff(STAR_VOIE_MOT, paramDriving);
Motion motion;
Trajectory trajectory;
CanMipMotor motionMotors[MOTION_MOTOR_MAX];
EncoderAB motionEncoders[MOTION_MOTOR_MAX];

// test odometrie sur roues motrice (tests sur cale par exemple)
#define TEST_ODO_MOT

static int star_robot_module_init()
{
	ax12.init("ax12", UART5_HALF_DUPLEX, 200000, AX12_MAX_ID, DYNAMIXEL_TYPE_AX12);
	//rx24.init("rx24", UART4_FULL_DUPLEX, 200000, RX24_MAX_ID, DYNAMIXEL_TYPE_RX24);

	leftWing.init(&ax12, AX12_LEFT_WING);
	rightWing.init(&ax12, AX12_RIGHT_WING);
	leftCarpet.init(&ax12, AX12_LEFT_CARPET);
	rightCarpet.init(&ax12, AX12_RIGHT_CARPET);
	lowFinger.init(&ax12, AX12_LOW_FINGER);
	highFinger.init(&ax12, AX12_HIGH_FINGER);
	rightFinger.init(&ax12, AX12_RIGHT_FINGER);
	leftFinger.init(&ax12, AX12_LEFT_FINGER);

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
#ifndef TEST_ODO_MOT
	location.init(&odoWheelKinematicsModelDiff);
#else
	log(LOG_ERROR, "Attention - orodemtrie sur roues motrices !!");
	location.init(&motorKinematicsModelDiff); // TESTS odo sur roues motirces
#endif
	detection.init(&hokuyo[0], &hokuyo[1], &location);

	motionEncoders[MOTION_MOTOR_LEFT].init(ENCODER_1, STAR_ODO1_WAY * 2 * M_PI * STAR_ODO1_WHEEL_RADIUS / (float)(STAR_ODO_ENCODER_RESOLUTION));
	motionEncoders[MOTION_MOTOR_RIGHT].init(ENCODER_2, STAR_ODO2_WAY * 2 * M_PI * STAR_ODO2_WHEEL_RADIUS / (float)(STAR_ODO_ENCODER_RESOLUTION));

	motionMotors[MOTION_MOTOR_LEFT].name = "moteur gauche";
	motionMotors[MOTION_MOTOR_LEFT].nodeId = CAN_MOTOR_LEFT_NODEID;
	motionMotors[MOTION_MOTOR_LEFT].inputGain = 60 * STAR_MOTOR_DRIVING1_RED / (float)(2 * M_PI * STAR_DRIVING1_WHEEL_RADIUS);
	motionMotors[MOTION_MOTOR_LEFT].outputGain = 2 * M_PI * STAR_DRIVING1_WHEEL_RADIUS / (float)(STAR_MOTOR_ENCODER_RESOLUTION * STAR_MOTOR_DRIVING1_RED);
	motionMotors[MOTION_MOTOR_LEFT].fault_disconnected_id = FAULT_CAN_MOTOR_DISCONNECTED_1;

	motionMotors[MOTION_MOTOR_RIGHT].name = "moteur droit";
	motionMotors[MOTION_MOTOR_RIGHT].nodeId = CAN_MOTOR_RIGHT_NODEID;
	motionMotors[MOTION_MOTOR_RIGHT].inputGain = 60 * STAR_MOTOR_DRIVING2_RED / (float)(2 * M_PI * STAR_DRIVING2_WHEEL_RADIUS);
	motionMotors[MOTION_MOTOR_RIGHT].outputGain = 2 * M_PI * STAR_DRIVING2_WHEEL_RADIUS / (float)(STAR_MOTOR_ENCODER_RESOLUTION * STAR_MOTOR_DRIVING2_RED);
	motionMotors[MOTION_MOTOR_RIGHT].fault_disconnected_id = FAULT_CAN_MOTOR_DISCONNECTED_0;

	for(int i = 0; i < MOTION_MOTOR_MAX; i++)
	{
		can_mip_register_node(&motionMotors[i]);
	}

#ifndef TEST_ODO_MOT
	motion.init(&detection, &location, &motorKinematicsModelDiff, &motionMotors[MOTION_MOTOR_LEFT], &motionMotors[MOTION_MOTOR_RIGHT], &motionEncoders[MOTION_MOTOR_LEFT], &motionEncoders[MOTION_MOTOR_RIGHT]);
#else
	motion.init(&detection, &location, &motorKinematicsModelDiff, &motionMotors[MOTION_MOTOR_LEFT], &motionMotors[MOTION_MOTOR_RIGHT], &motionMotors[MOTION_MOTOR_LEFT], &motionMotors[MOTION_MOTOR_RIGHT]);
#endif
	trajectory.init(&detection, &motion, &location, linearParam, angularParam);

	return 0;
}

module_init(star_robot_module_init, INIT_MAIN_ROBOT);
