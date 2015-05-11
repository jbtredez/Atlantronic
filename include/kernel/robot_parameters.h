#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H

//! @file robot_parameters.h
//! @brief Parameters
//! @author Atlantronic

#include <math.h>

#define PARAM_LEFT_CORNER_X                    125
#define PARAM_LEFT_CORNER_Y                    155
#define PARAM_RIGHT_CORNER_X                   125
#define PARAM_RIGHT_CORNER_Y                  -155
#define PARAM_NP_X                            -125

#define DRIVING1_WHEEL_RADIUS                 31.1
#define DRIVING2_WHEEL_RADIUS                 31.1
#define ODO1_WHEEL_RADIUS                     39.6
#define ODO2_WHEEL_RADIUS                     39.6
#define ODO1_WAY                                -1
#define ODO2_WAY                                 1

#define VOIE_MOT                            290.0f
#define VOIE_MOT_INV                    1/VOIE_MOT
#define VOIE_ODO                            218.2f
#define VOIE_ODO_INV                    1/VOIE_ODO

#define MOTOR_RED                             5.2f
#define MOTOR_ENCODER_RESOLUTION              1024
#define ODO_ENCODER_RESOLUTION                4096

#define MOTOR_DRIVING1_RED              -MOTOR_RED  //!< reduction moteur 1
#define MOTOR_DRIVING2_RED               MOTOR_RED  //!< reduction moteur 2

#define LEFT_WHEEL                               0
#define RIGHT_WHEEL                              1

#define REAR_OMRON_RANGE                       400

#endif
