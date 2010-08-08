#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H

//! @file robot_parameters.h
//! @brief Parameters
//! @author Jean-Baptiste Trédez

#include <math.h>

#define PARAM_RIGHT_ODO_WHEEL_RADIUS          50.0f
#define PARAM_LEFT_ODO_WHEEL_RADIUS           50.0f

#define PARAM_RIGHT_ODO_WHEEL_WAY                 1
#define PARAM_LEFT_ODO_WHEEL_WAY                 -1

#define PARAM_VOIE_ODO                       200.0f

#define PARAM_ENCODERS_RES                 (4096*4)
#define PARAM_DIST_ODO_GAIN                (M_PI / PARAM_ENCODERS_RES)
#define PARAM_ROT_ODO_GAIN                 (2.0f * M_PI / (PARAM_ENCODERS_RES * PARAM_VOIE_ODO))

#endif
