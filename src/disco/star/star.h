#ifndef STAR_H
#define STAR_H

#include "disco/bot.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/driver/DynamixelManager.h"
#include "kernel/CanMipMotor.h"

// Dimensions
#define STAR_HALF_LENGTH							110
#define STAR_HALF_WIDTH								100
#define STAR_REAR_OMRON_RANGE                       400

// Numero de moteur
#define STAR_LEFT_WHEEL                             0
#define STAR_RIGHT_WHEEL                            1

// Gains d'asservissement
// Gains d'asservissement
#define STAR_XKP									2
#define STAR_XKI									0
#define STAR_XKD									0
#define STAR_XMAX									100 // TODO voir saturation

#define STAR_YKP									0
#define STAR_YKI									0
#define STAR_YKD									0
#define STAR_YMAX									1 // TODO voir saturation + regler

#define STAR_THETAKP								15
#define STAR_THETAKI								0
#define STAR_THETAKD								0
#define STAR_THETAMAX								1 // TODO voir saturation

// Parametres des roues
#define STAR_VOIE_MOT                            164.0f
#define STAR_VOIE_ODO_POS                        112.9f // sens + : , sens - : 112.45
#define STAR_VOIE_ODO_NEG                       112.45f // sens + : 112.9, sens - : 112.45
#define STAR_DRIVING1_WHEEL_RADIUS               100.0f
#define STAR_DRIVING2_WHEEL_RADIUS               100.0f
#define STAR_MOTOR_RED                   (5.2*88/25.0f)
#define STAR_MOTOR_ENCODER_RESOLUTION              1024
#define STAR_MOTOR_DRIVING1_RED         -STAR_MOTOR_RED  //!< reduction moteur 1
#define STAR_MOTOR_DRIVING2_RED          STAR_MOTOR_RED  //!< reduction moteur 2
#define STAR_ODO1_WHEEL_RADIUS                    38.25f
#define STAR_ODO2_WHEEL_RADIUS                    38.5f
#define STAR_ODO1_WAY                                 1
#define STAR_ODO2_WAY                                -1
#define STAR_ODO_ENCODER_RESOLUTION                4096

// Parametres lasers
#define STAR_HOKUYO_1_X                             0.00
#define STAR_HOKUYO_1_Y                             0.00
#define STAR_HOKUYO_1_THETA                      -M_PI_2

#define STAR_HOKUYO_2_X                             0.00
#define STAR_HOKUYO_2_Y                             0.00
#define STAR_HOKUYO_2_THETA                         0.00

// position initiale estimee (endroit ou on pose le robot pour le recalage) couleur bleu
#define STAR_INIT_POS_X                           600.00
#define STAR_INIT_POS_Y                           800.00
#define STAR_INIT_POS_THETA                      -M_PI_2

#ifndef LINUX
extern Location location;
extern DynamixelManager ax12;
extern Dynamixel leftFishWing;
extern Dynamixel leftFishRemover;
extern Dynamixel rightFishWing;
extern Dynamixel rightFishRemover;
extern Dynamixel leftDoor;
extern Dynamixel rightDoor;
extern Dynamixel towerPliers;
extern Dynamixel towerPliersTidier;
extern Dynamixel parasol;
#endif

#endif
