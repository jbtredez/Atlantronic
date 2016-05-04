#include "kernel/module.h"
#include "star.h"
#include "kernel/driver/hokuyo.h"
#include "kernel/driver/rplidar.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/kinematics_model/KinematicsModelDiff.h"
#include "kernel/driver/encoder/EncoderAB.h"
#include "kernel/log.h"
#include "disco/star/servos.h"

KinematicsParameters paramDriving = {1800, 1500, 1500};
KinematicsParameters linearParam = {500, 500, 500};
KinematicsParameters angularParam = {5, 5, 5}; // ne pas modifier, robot calibre pour cela ! (bug odometrie calibre selon sens de rotation et vitesse)

//Hokuyo hokuyo[HOKUYO_MAX];
Rplidar rplidar;
Dynamixel leftFishWing;
Dynamixel leftFishRemover;
Dynamixel rightFishWing;
Dynamixel rightFishRemover;
Dynamixel leftDoor;
Dynamixel rightDoor;
Dynamixel towerPliers;
Dynamixel towerPliersTidier;
Dynamixel parasol;
DynamixelManager ax12;
//DynamixelManager rx24;
Bot PramBot;
Location location;
Detection detection;
KinematicsModelDiff odoWheelKinematicsModelDiff(STAR_VOIE_ODO_POS, STAR_VOIE_ODO_NEG, paramDriving);
KinematicsModelDiff motorKinematicsModelDiff(STAR_VOIE_MOT, STAR_VOIE_MOT, paramDriving);
//Motion motion;
//Trajectory trajectory;
CanMipMotor motionMotors[MOTION_MOTOR_MAX];
EncoderAB motionEncoders[MOTION_MOTOR_MAX];

// test odometrie sur roues motrice (tests sur cale par exemple)
//#define TEST_ODO_MOT

static int star_robot_module_init()
{
	PramBot.init();

	Bot::halfLength = STAR_HALF_LENGTH;
	Bot::halfWidth = STAR_HALF_WIDTH;
	Bot::rearOmronRange = STAR_REAR_OMRON_RANGE;
	Bot::leftWheel = STAR_LEFT_WHEEL;
	Bot::rightWheel = STAR_RIGHT_WHEEL;
	Bot::xKP = STAR_XKP;
	Bot::xKI = STAR_XKI;
	Bot::xKD = STAR_XKD;
	Bot::xMax = STAR_XMAX;
	Bot::yKP = STAR_YKP;
	Bot::yKI = STAR_YKI;
	Bot::yKD = STAR_YKD;
	Bot::yMax = STAR_YMAX;
	Bot::tethaKP = STAR_THETAKP;
	Bot::tethaKI = STAR_THETAKI;
	Bot::tethaKD = STAR_THETAKD;
	Bot::tethaMax = STAR_THETAMAX;
	Bot::voieMot = STAR_VOIE_MOT;
	//Bot::voieOdo = STAR_VOIE_ODO; // TODO
	Bot::driving1WheelRadius = STAR_DRIVING1_WHEEL_RADIUS;
	Bot::driving2WheelRadius = STAR_DRIVING2_WHEEL_RADIUS;
	Bot::motorDriving1Red = STAR_MOTOR_DRIVING1_RED;
	Bot::motorDriving2Red = STAR_MOTOR_DRIVING2_RED;
	Bot::motorRpmToVolt = 1;	 // unused
	Bot::odo1WheelRadius = STAR_ODO1_WHEEL_RADIUS;
	Bot::odo2WheelRadius = STAR_ODO2_WHEEL_RADIUS;
	Bot::odo1Way = STAR_ODO1_WAY;
	Bot::odo2Way = STAR_ODO2_WAY;
	Bot::odoEncoderResolution = STAR_ODO_ENCODER_RESOLUTION;

	ax12.init("ax12", UART5_HALF_DUPLEX, 200000, AX12_MAX_ID, DYNAMIXEL_TYPE_AX12);
	//rx24.init("rx24", UART4_FULL_DUPLEX, 200000, RX24_MAX_ID, DYNAMIXEL_TYPE_RX24);

	leftFishWing.init(&ax12, AX12_STAR_LEFT_FISH_WING);
	leftFishRemover.init(&ax12, AX12_STAR_LEFT_FISH_REMOVER);
	rightFishWing.init(&ax12, AX12_STAR_RIGHT_FISH_WING);
	rightFishRemover.init(&ax12, AX12_STAR_RIGHT_FISH_REMOVER);
	leftDoor.init(&ax12, AX12_STAR_LEFT_DOOR);
	rightDoor.init(&ax12, AX12_STAR_RIGHT_DOOR);
	towerPliers.init(&ax12, AX12_STAR_TOWER_PLIERS);
	towerPliersTidier.init(&ax12, AX12_STAR_TOWER_PLIERS_TIDIER);
	parasol.init(&ax12, AX12_STAR_PARASOL);

	Servos::setTorque(true);


//	hokuyo[0].init(USART3_FULL_DUPLEX, "hokuyo1", HOKUYO1, &location);
//	hokuyo[0].setPosition(VectPlan( 0, 0, 0), 1);
//	hokuyo[0].scan.theta_min = -M_PI;
//	hokuyo[0].scan.theta_max = M_PI;
//	hokuyo[0].scan.min_object_size = 1;
//	hokuyo[0].scan.min_distance = 125;
//
//	hokuyo[1].init(USART1_FULL_DUPLEX, "hokuyo2", HOKUYO2, &location);
//	hokuyo[1].setPosition(VectPlan(0, 0, 0), 1);
//	hokuyo[1].scan.theta_min = -M_PI;
//	hokuyo[1].scan.theta_max = M_PI;
//	hokuyo[1].scan.min_object_size = 1;
//	hokuyo[1].scan.min_distance = 100;

	rplidar.init(USART6_FULL_DUPLEX, "rplidar", &location);
	rplidar.scan.min_object_size = 1;


#ifndef TEST_ODO_MOT
	location.init(&odoWheelKinematicsModelDiff);
#else
	log(LOG_ERROR, "Attention - orodemtrie sur roues motrices !!");
	location.init(&motorKinematicsModelDiff); // TESTS odo sur roues motirces
#endif

	detection.init(&rplidar, &location);

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
