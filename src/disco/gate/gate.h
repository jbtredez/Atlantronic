#ifndef GATE_H
#define GATE_H

#include "disco/bot.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/driver/DynamixelManager.h"
#include "kernel/driver/StepperDriver.h"
#include "kernel/PwmMotor.h"

// Dimensions
#define GATE_HALF_LENGTH                        112.50f
#define GATE_HALF_WIDTH                          90.00f
#define GATE_NP_X                               -82.50f
#define GATE_REAR_OMRON_RANGE                       400

// Numero de moteur
#define GATE_LEFT_WHEEL                             0
#define GATE_RIGHT_WHEEL                            1

// Gains d'asservissement
#define GATE_XKP									10
#define GATE_XKI									5
#define GATE_XKD									0
#define GATE_XMAX									100 // TODO voir saturation

#define GATE_YKP									0
#define GATE_YKI									0
#define GATE_YKD									0
#define GATE_YMAX									1 // TODO voir saturation + regler

#define GATE_THETAKP								10
#define GATE_THETAKI								50
#define GATE_THETAKD								0
#define GATE_THETAMAX								1 // TODO voir saturation

// Parametres des roues
#define GATE_VOIE_MOT                           122.32f
#define GATE_VOIE_ODO_POSITIF                    77.20f		// VOIE
#define GATE_VOIE_ODO_NEGATIF                    77.20f
#define GATE_DRIVING1_WHEEL_RADIUS               30.16f
#define GATE_DRIVING2_WHEEL_RADIUS               30.16f
#define GATE_MOTOR_DRIVING1_RED                 -19.00f  //!< reduction moteur 1
#define GATE_MOTOR_DRIVING2_RED                  19.00f  //!< reduction moteur 2
#define GATE_MOTOR_RPM_TO_VOLT             (24.0f/9842.0f)
#define GATE_ODO1_WHEEL_RADIUS                   29.08f		// ROUES
#define GATE_ODO2_WHEEL_RADIUS                   29.08f
#define GATE_ODO1_WAY                                -1
#define GATE_ODO2_WAY                                 1
#define GATE_ODO_ENCODER_RESOLUTION                4096

#define GATE_STEPPER_MOTOR_STEP_BY_TURN             200

// Parametres lasers
#define GATE_HOKUYO_1_X                             0.00
#define GATE_HOKUYO_1_Y                             0.00
#define GATE_HOKUYO_1_THETA                      -M_PI_2

#define GATE_HOKUYO_2_X                             0.00
#define GATE_HOKUYO_2_Y                             0.00
#define GATE_HOKUYO_2_THETA                         0.00

// position initiale estimee (endroit ou on pose le robot pour le recalage) couleur bleu
#define GATE_INIT_POS_X                          1450.00
#define GATE_INIT_POS_Y                           820.00
#define GATE_INIT_POS_THETA                         M_PI

#ifndef LINUX
extern Location location;
extern Motion motion;
extern Trajectory trajectory;
extern DynamixelManager ax12;
extern Dynamixel parasol;
extern StepperDriver cylinder;
#endif

struct GateCmdSetMotorsPidArg
{
	float kp1;
	float ki1;
	float kd1;
	float kp2;
	float ki2;
	float kd2;
}  __attribute__((packed));

#endif
