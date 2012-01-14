#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H

//! @file robot_parameters.h
//! @brief Parameters
//! @author Atlantronic

#include <math.h>

#define PI                              3.14159265f

#define PARAM_RIGHT_ODO_WHEEL_RADIUS          40.1f
#define PARAM_LEFT_ODO_WHEEL_RADIUS           40.1f
#define PARAM_RIGHT_MOT_WHEEL_RADIUS          50.0f
#define PARAM_LEFT_MOT_WHEEL_RADIUS           50.0f

#define PARAM_RIGHT_ODO_WHEEL_WAY                 1
#define PARAM_LEFT_ODO_WHEEL_WAY                  1
#define PARAM_RIGHT_MOT_WHEEL_WAY                 1
#define PARAM_LEFT_MOT_WHEEL_WAY                  1

#define PARAM_VOIE_ODO                       301.15f
#define PARAM_VOIE_MOT                       120.0f

#define PARAM_ENCODERS_RES                     4096

#define PARAM_MOT_RED                            21
#define PARAM_ODO_RED                             1

#define PARAM_DIST_ODO_GAIN                (float) (PI / (PARAM_ODO_RED * PARAM_ENCODERS_RES) )
#define PARAM_ROT_ODO_GAIN                 (float) (2.0f * PI / (PARAM_ENCODERS_RES * PARAM_VOIE_ODO * PARAM_ODO_RED))
#define PARAM_DIST_MOD_GAIN                (float) (1.0f / (2.0f * PARAM_MOT_RED))
#define PARAM_ROT_MOD_GAIN                 (float) (1.0f / (PARAM_VOIE_MOT * PARAM_MOT_RED))

#define PARAM_LEFT_CORNER_X                    155
#define PARAM_LEFT_CORNER_Y                    175
#define PARAM_RIGHT_CORNER_X                   155
#define PARAM_RIGHT_CORNER_Y                  -175

#define PARAM_NP_X                          -95.0f

#endif
