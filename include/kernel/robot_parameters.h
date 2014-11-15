#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H

//! @file robot_parameters.h
//! @brief Parameters
//! @author Atlantronic

#include <math.h>

#define PARAM_LEFT_CORNER_X                    190
#define PARAM_LEFT_CORNER_Y                    175
#define PARAM_RIGHT_CORNER_X                   190
#define PARAM_RIGHT_CORNER_Y                  -175
#define PARAM_NP_X                            -103

#define DRIVING1_WHEEL_RADIUS       33
#define DRIVING2_WHEEL_RADIUS       33
#define DRIVING3_WHEEL_RADIUS       33

#define MOTOR_RED                   676/49.0f  //!< reducteur moteur faulhaber
#define MOTOR_DRIVING1_RED          MOTOR_RED  //!< reduction moteur 1
#define MOTOR_DRIVING2_RED          MOTOR_RED
#define MOTOR_DRIVING3_RED          MOTOR_RED
#define MOTOR_STEERING1_RED         (4.375f*MOTOR_RED)
#define MOTOR_STEERING2_RED         (4.375f*MOTOR_RED)
#define MOTOR_STEERING3_RED         (4.375f*MOTOR_RED)

#define MOTOR_STEERING1_OFFSET       (5.8471f/4.375f)
#define MOTOR_STEERING2_OFFSET          (7.0f/4.375f)
#define MOTOR_STEERING3_OFFSET         (-4.5f/4.375f)

#define MOTOR_ENCODER_RESOLUTION         3000

#endif
