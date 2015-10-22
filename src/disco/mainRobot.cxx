#include "kernel/module.h"
#include "mainRobot.h"
#include "kernel/driver/hokuyo.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/kinematics_model/KinematicsModelDiff.h"


Location location;
Hokuyo hokuyo[HOKUYO_MAX];
Detection detection;
KinematicsModelDiff kinematicsModelDiff;
Motion motion;
Trajectory trajectory;

static int main_robot_module_init()
{
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
	location.init(&kinematicsModelDiff);
	detection.init(&hokuyo[0], &hokuyo[1], &location);
	motion.init(&detection, &location, &kinematicsModelDiff);
	trajectory.init(&detection, &motion, &location);

	return 0;
}

module_init(main_robot_module_init, INIT_MAIN_ROBOT);
