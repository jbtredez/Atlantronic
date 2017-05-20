#include "kernel/module.h"
#include "gate.h"
#include "kernel/driver/hokuyo.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/kinematics_model/KinematicsModelDiff.h"
#include "kernel/driver/encoder/EncoderSimulFromKinematicsModel.h"
#include "kernel/driver/encoder/EncoderAB.h"
#include "kernel/control.h"

KinematicsParameters paramDriving = {1600, 1500, 1500};
KinematicsParameters linearParam = {500, 800, 800};
KinematicsParameters angularParam = {2, 5, 5};

Hokuyo hokuyo[HOKUYO_MAX];
Dynamixel missileLeft;
Dynamixel missileRight;
Dynamixel pusher;
Dynamixel clampLeft;
Dynamixel clampRight;

DynamixelManager ax12;
//DynamixelManager rx24;

Bot PramBot;
Location location;
Detection detection;
KinematicsModelDiff odoWheelKinematicsModelDiff(GATE_VOIE_ODO_POSITIF, GATE_VOIE_ODO_NEGATIF, paramDriving);
KinematicsModelDiff motorKinematicsModelDiff(GATE_VOIE_MOT, GATE_VOIE_MOT, paramDriving);
//Motion motion;
//Trajectory trajectory;
PwmMotor motionMotors[MOTION_MOTOR_MAX];
EncoderSimulFromKinematicsModel motionMotorEncoder[MOTION_MOTOR_MAX];
EncoderAB motionEncoders[MOTION_MOTOR_MAX];
StepperDriver cylinder;

static void cmd_set_motors_pid(void* arg, void* data);
static void cmd_rotate_cylinder_to(void* arg, void* data);

static int gate_robot_module_init()
{
	PramBot.init();
	Bot::halfLength = GATE_HALF_LENGTH;
	Bot::halfWidth = GATE_HALF_WIDTH;
	Bot::rearOmronRange = GATE_REAR_OMRON_RANGE;
	Bot::leftWheel = GATE_LEFT_WHEEL;
	Bot::rightWheel = GATE_RIGHT_WHEEL;
	Bot::xKP = GATE_XKP;
	Bot::xKI = GATE_XKI;
	Bot::xKD = GATE_XKD;
	Bot::xMax = GATE_XMAX;
	Bot::yKP = GATE_YKP;
	Bot::yKI = GATE_YKI;
	Bot::yKD = GATE_YKD;
	Bot::yMax = GATE_YMAX;
	Bot::tethaKP = GATE_THETAKP;
	Bot::tethaKI = GATE_THETAKI;
	Bot::tethaKD = GATE_THETAKD;
	Bot::tethaMax = GATE_THETAMAX;
	Bot::voieMot = GATE_VOIE_MOT;
	//Bot::voieOdo = GATE_VOIE_ODO;
	Bot::driving1WheelRadius = GATE_DRIVING1_WHEEL_RADIUS;
	Bot::driving2WheelRadius = GATE_DRIVING2_WHEEL_RADIUS;
	Bot::motorDriving1Red = GATE_MOTOR_DRIVING1_RED;
	Bot::motorDriving2Red = GATE_MOTOR_DRIVING2_RED;
	Bot::motorRpmToVolt = GATE_MOTOR_RPM_TO_VOLT;
	Bot::odo1WheelRadius = GATE_ODO1_WHEEL_RADIUS;
	Bot::odo2WheelRadius = GATE_ODO2_WHEEL_RADIUS;
	Bot::odo1Way = GATE_ODO1_WAY;
	Bot::odo2Way = GATE_ODO2_WAY;
	Bot::odoEncoderResolution = GATE_ODO_ENCODER_RESOLUTION;


	ax12.init("ax12", UART5_HALF_DUPLEX, 200000, AX12_MAX_ID, DYNAMIXEL_TYPE_AX12);
	//rx24.init("rx24", UART4_FULL_DUPLEX, 200000, RX24_MAX_ID, DYNAMIXEL_TYPE_RX24);

	cylinder.init(IO_CYLINDER_STEP, IO_CYLINDER_DIR, GATE_STEPPER_MOTOR_STEP_BY_TURN / (2*M_PI), 4*M_PI, 8*M_PI, 8*M_PI);

	// Init AX12
	missileLeft.init(&ax12, AX12_GATE_MISSILE_LEFT);
	missileRight.init(&ax12, AX12_GATE_MISSILE_LEFT);
	pusher.init(&ax12, AX12_GATE_PUSHER);
	clampLeft.init(&ax12, AX12_GATE_CLAMP_LEFT);
	clampRight.init(&ax12, AX12_GATE_CLAMP_RIGHT);

	// Set torque value
	missileLeft.setTorqueLimit(0.85);
	missileRight.setTorqueLimit(0.85);
	pusher.setTorqueLimit(0.85);

	// Set AX12 limit angles
	missileLeft.setGoalLimits( -M_PI_2, 0 );
	missileRight.setGoalLimits( 0, M_PI_2 );
	pusher.setGoalLimits( -M_PI_2, 0 );


	pusher.setTorqueEnable(1);



	hokuyo[0].init(USART3_FULL_DUPLEX, "hokuyo1", HOKUYO1, &location);
	hokuyo[0].setPosition(VectPlan( GATE_HOKUYO_1_X, GATE_HOKUYO_1_Y, GATE_HOKUYO_1_THETA), 1);
	hokuyo[0].scan.theta_min = -M_PI;
	hokuyo[0].scan.theta_max = M_PI;
	hokuyo[0].scan.min_object_size = 1;
	hokuyo[0].scan.min_distance = 125;
/*
	hokuyo[1].init(USART1_FULL_DUPLEX, "hokuyo2", HOKUYO2, &location);
	hokuyo[1].setPosition(VectPlan(GATE_HOKUYO_2_X, GATE_HOKUYO_2_Y, GATE_HOKUYO_2_THETA), 1);
	hokuyo[1].scan.theta_min = -M_PI;
	hokuyo[1].scan.theta_max = M_PI;
	hokuyo[1].scan.min_object_size = 1;
	hokuyo[1].scan.min_distance = 100;
*/
	location.init(&odoWheelKinematicsModelDiff);
	detection.init(&hokuyo[0], &hokuyo[1], &location);

	motionMotorEncoder[MOTION_MOTOR_LEFT].init(&location, &motorKinematicsModelDiff, Bot::leftWheel);
	motionMotorEncoder[MOTION_MOTOR_RIGHT].init(&location, &motorKinematicsModelDiff, Bot::rightWheel);
	motionEncoders[MOTION_MOTOR_LEFT].init(ENCODER_1, Bot::odo1Way * 2 * M_PI * Bot::odo1WheelRadius / (float)(Bot::odoEncoderResolution ));
	motionEncoders[MOTION_MOTOR_RIGHT].init(ENCODER_2, Bot::odo2Way * 2 * M_PI * Bot::odo2WheelRadius / (float)(Bot::odoEncoderResolution ));

	motionMotors[MOTION_MOTOR_LEFT].name = "moteur gauche";
	motionMotors[MOTION_MOTOR_LEFT].pwmId = PWM_1;
	motionMotors[MOTION_MOTOR_LEFT].inputGain = 60 * GATE_MOTOR_DRIVING1_RED / (float)(2 * M_PI * GATE_DRIVING1_WHEEL_RADIUS) * GATE_MOTOR_RPM_TO_VOLT;
	motionMotors[MOTION_MOTOR_LEFT].encoder = &motionMotorEncoder[MOTION_MOTOR_LEFT];
	motionMotors[MOTION_MOTOR_LEFT].pid.init(1, 0, 0, 1000);

	motionMotors[MOTION_MOTOR_RIGHT].name = "moteur droit";
	motionMotors[MOTION_MOTOR_RIGHT].pwmId = PWM_2;
	motionMotors[MOTION_MOTOR_RIGHT].inputGain = 60 * GATE_MOTOR_DRIVING2_RED / (float)(2 * M_PI * GATE_DRIVING2_WHEEL_RADIUS) * GATE_MOTOR_RPM_TO_VOLT;
	motionMotors[MOTION_MOTOR_RIGHT].encoder = &motionMotorEncoder[MOTION_MOTOR_RIGHT];
	motionMotors[MOTION_MOTOR_RIGHT].pid.init(1, 0, 0, 1000);

	motion.init(&detection, &location, &motorKinematicsModelDiff, &motionMotors[MOTION_MOTOR_LEFT], &motionMotors[MOTION_MOTOR_RIGHT], &motionEncoders[MOTION_MOTOR_LEFT], &motionEncoders[MOTION_MOTOR_RIGHT]);
	trajectory.init(&detection, &motion, &location, linearParam, angularParam);

	usb_add_cmd(USB_CMD_SET_MOTORS_PID, &cmd_set_motors_pid, NULL);
	usb_add_cmd(USB_CMD_CYLINDER, &cmd_rotate_cylinder_to, NULL);

	return 0;
}

module_init(gate_robot_module_init, INIT_MAIN_ROBOT);

void cmd_set_motors_pid(void* /*arg*/, void* data)
{
	GateCmdSetMotorsPidArg* val = (GateCmdSetMotorsPidArg*) data;
	log_format(LOG_INFO, "left  : kp %d ki %d kd %d", (int)(val->kp1), (int)(val->ki1), (int)(val->kd1));
	log_format(LOG_INFO, "right : kp %d ki %d kd %d", (int)(val->kp2), (int)(val->ki2), (int)(val->kd2));
	motionMotors[MOTION_MOTOR_LEFT].pid.init(val->kp1, val->ki1, val->kd1, 1000);
	motionMotors[MOTION_MOTOR_RIGHT].pid.init(val->kp2, val->ki2, val->kd2, 1000);
}

void cmd_rotate_cylinder_to(void* /*arg*/, void* data)
{
	float val = *((float*) data);
	cylinder.setPosition(val);
}
