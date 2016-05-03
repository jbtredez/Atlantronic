#ifndef GATE_H
#define GATE_H

#include "disco/bot.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/driver/DynamixelManager.h"
#include "kernel/PwmMotor.h"

// Dimensions
#define GATE_HALF_LENGTH							90
#define GATE_HALF_WIDTH								90
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
#define GATE_VOIE_MOT                            144.0f
#define GATE_VOIE_ODO                             92.37f
#define GATE_DRIVING1_WHEEL_RADIUS                90.1f
#define GATE_DRIVING2_WHEEL_RADIUS                90.8f
#define GATE_MOTOR_DRIVING1_RED             -(78/10.0f)  //!< reduction moteur 1
#define GATE_MOTOR_DRIVING2_RED              (78/10.0f)  //!< reduction moteur 2
#define GATE_MOTOR_RPM_TO_VOLT               (3/163.5f)
#define GATE_ODO1_WHEEL_RADIUS                    38.89f
#define GATE_ODO2_WHEEL_RADIUS                    38.89f
#define GATE_ODO1_WAY                                 1
#define GATE_ODO2_WAY                                -1
#define GATE_ODO_ENCODER_RESOLUTION                4096

#ifndef LINUX
extern Location location;
extern Motion motion;
extern Trajectory trajectory;
extern DynamixelManager ax12;
#endif

#endif
