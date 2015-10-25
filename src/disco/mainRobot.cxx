#include "kernel/module.h"
#include "mainRobot.h"
#include "kernel/driver/hokuyo.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/kinematics_model/KinematicsModelDiff.h"
#include "robot_parameters.h"

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
KinematicsModelDiff odoWheelKinematicsModelDiff(VOIE_ODO);
KinematicsModelDiff motorKinematicsModelDiff(VOIE_MOT);
Motion motion;
Trajectory trajectory;

static int main_robot_module_init()
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
	motion.init(&detection, &location, &motorKinematicsModelDiff);
	trajectory.init(&detection, &motion, &location);

	return 0;
}

module_init(main_robot_module_init, INIT_MAIN_ROBOT);
