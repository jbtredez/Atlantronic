#include "kernel/module.h"
#include "gate.h"
#include "kernel/driver/hokuyo.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/kinematics_model/KinematicsModelDiff.h"
#include "robot_parameters.h"

#define GATE_VOIE_MOT                            164.0f
#define GATE_VOIE_ODO                            110.0f
#define GATE_DRIVING1_WHEEL_RADIUS                90.0f
#define GATE_DRIVING2_WHEEL_RADIUS                90.0f
#define GATE_MOTOR_DRIVING1_RED              -(78/10.0f)  //!< reduction moteur 1
#define GATE_MOTOR_DRIVING2_RED               (78/10.0f)  //!< reduction moteur 2
#define GATE_MOTOR_RPM_TO_VOLT                 (1/163.5f)


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
KinematicsModelDiff odoWheelKinematicsModelDiff(GATE_VOIE_ODO);
KinematicsModelDiff motorKinematicsModelDiff(GATE_VOIE_MOT);
Motion motion;
Trajectory trajectory;
PwmMotor motionMotors[MOTION_MOTOR_MAX];

static int gate_robot_module_init()
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
	location.init(&odoWheelKinematicsModelDiff);
	detection.init(&hokuyo[0], &hokuyo[1], &location);

	motionMotors[MOTION_MOTOR_RIGHT].name = "moteur droit";
	motionMotors[MOTION_MOTOR_RIGHT].pwmId = 0;
	motionMotors[MOTION_MOTOR_RIGHT].inputGain = 60 * GATE_MOTOR_DRIVING2_RED / (float)(2 * M_PI * GATE_DRIVING2_WHEEL_RADIUS) * GATE_MOTOR_RPM_TO_VOLT;

	motionMotors[MOTION_MOTOR_LEFT].name = "moteur gauche";
	motionMotors[MOTION_MOTOR_LEFT].pwmId = 1;
	motionMotors[MOTION_MOTOR_LEFT].inputGain = 60 * GATE_MOTOR_DRIVING1_RED / (float)(2 * M_PI * GATE_DRIVING1_WHEEL_RADIUS) * GATE_MOTOR_RPM_TO_VOLT;

	motion.init(&detection, &location, &motorKinematicsModelDiff, &motionMotors[MOTION_MOTOR_LEFT], &motionMotors[MOTION_MOTOR_RIGHT]);
	trajectory.init(&detection, &motion, &location);

	return 0;
}

module_init(gate_robot_module_init, INIT_MAIN_ROBOT);
