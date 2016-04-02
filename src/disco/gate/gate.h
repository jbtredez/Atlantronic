#ifndef GATE_H
#define GATE_H

#include "middleware/trajectory/Trajectory.h"
#include "kernel/driver/DynamixelManager.h"
#include "kernel/PwmMotor.h"

#define GATE_VOIE_MOT                            144.0f
#define GATE_VOIE_ODO                             90.0f
#define GATE_DRIVING1_WHEEL_RADIUS                90.0f
#define GATE_DRIVING2_WHEEL_RADIUS                90.0f
#define GATE_MOTOR_DRIVING1_RED             -(78/10.0f)  //!< reduction moteur 1
#define GATE_MOTOR_DRIVING2_RED              (78/10.0f)  //!< reduction moteur 2
#define GATE_MOTOR_RPM_TO_VOLT               (3/163.5f)

#define GATE_ODO1_WHEEL_RADIUS                    39.7f
#define GATE_ODO2_WHEEL_RADIUS                    39.7f
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
