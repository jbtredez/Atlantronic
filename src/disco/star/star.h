#ifndef STAR_H
#define STAR_H

#include "middleware/trajectory/Trajectory.h"
#include "kernel/driver/DynamixelManager.h"
#include "kernel/CanMipMotor.h"

#define STAR_VOIE_MOT                            164.0f
#define STAR_VOIE_ODO                            110.0f
#define STAR_DRIVING1_WHEEL_RADIUS               100.0f
#define STAR_DRIVING2_WHEEL_RADIUS               100.0f
#define STAR_MOTOR_RED                   (5.2*88/25.0f)
#define STAR_MOTOR_ENCODER_RESOLUTION              1024
#define STAR_MOTOR_DRIVING1_RED         -STAR_MOTOR_RED  //!< reduction moteur 1
#define STAR_MOTOR_DRIVING2_RED          STAR_MOTOR_RED  //!< reduction moteur 2

#define STAR_ODO1_WHEEL_RADIUS                    39.7f
#define STAR_ODO2_WHEEL_RADIUS                    39.7f
#define STAR_ODO1_WAY                                 1
#define STAR_ODO2_WAY                                -1
#define STAR_ODO_ENCODER_RESOLUTION                4096

#ifndef LINUX
extern Location location;
extern Motion motion;
extern Trajectory trajectory;
extern DynamixelManager ax12;
extern Dynamixel leftWing;
extern Dynamixel rightWing;
extern Dynamixel leftCarpet;
extern Dynamixel rightCarpet;
extern Dynamixel lowFinger;
extern Dynamixel highFinger;
extern Dynamixel rightFinger;
extern Dynamixel leftFinger;
#endif

#endif
