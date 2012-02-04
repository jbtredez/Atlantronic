#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H

//! @file robot_parameters.h
//! @brief Parameters
//! @author Atlantronic

#include <math.h>

#define PI                              3.141592654f
#define PI_FX29                           1686629713      //!< pi en 2^-29 rd

#define PARAM_RIGHT_ODO_WHEEL_RADIUS_FX      2627994      //!< rayon de la roue droite en 2^-16 mm (virgule fixe) ( = 40.1 mm)
#define PARAM_LEFT_ODO_WHEEL_RADIUS_FX       2627994      //!< rayon de la roue droite en 2^-16 mm (virgule fixe) ( = 40.1 mm)

#define PARAM_INVERTED_VOIE_FX39          1825521547      //!< inverse de la voie en 2^-39 mm^-1 ( = 1 / 301.15mm)

#define PARAM_RIGHT_ODO_WHEEL_RADIUS          40.1000061f
#define PARAM_LEFT_ODO_WHEEL_RADIUS           40.1000061f
#define PARAM_RIGHT_MOT_WHEEL_RADIUS          50.0f
#define PARAM_LEFT_MOT_WHEEL_RADIUS           50.0f

#define PARAM_RIGHT_ODO_WHEEL_WAY                 1
#define PARAM_LEFT_ODO_WHEEL_WAY                  1
#define PARAM_RIGHT_MOT_WHEEL_WAY                 1
#define PARAM_LEFT_MOT_WHEEL_WAY                  1

#define PARAM_VOIE_ODO                       301.1526835f
#define PARAM_VOIE_MOT                       120.0f

#define PARAM_ENCODERS_RES                     4096
#define PARAM_ENCODERS_BIT_RES                   12

#define PARAM_MOT_RED                            21
#define PARAM_ODO_RED                             1

#define PARAM_DIST_ODO_GAIN                (float) (PI / (PARAM_ODO_RED * PARAM_ENCODERS_RES) )
#define PARAM_ROT_ODO_GAIN                 (float) (2.0f * PI / (PARAM_ENCODERS_RES * PARAM_VOIE_ODO * PARAM_ODO_RED))
#define PARAM_DIST_MOD_GAIN                (float) (1.0f / (2.0f * PARAM_MOT_RED))
#define PARAM_ROT_MOD_GAIN                 (float) (1.0f / (PARAM_VOIE_MOT * PARAM_MOT_RED))

#define PARAM_LEFT_CORNER_X                    155*65536
#define PARAM_LEFT_CORNER_Y                   -175*65536
#define PARAM_RIGHT_CORNER_X                   155*65536
#define PARAM_RIGHT_CORNER_Y                   175*65536

#define PARAM_NP_X                          -95.0f

#define PARAM_FOO_HOKUYO_X                     130*65536
#define PARAM_FOO_HOKUYO_Y                     160*65536
#define PARAM_FOO_HOKUYO_ALPHA              (1 << 23)
#define PARAM_FOO_HOKUYO_SENS                   -1

#define PARAM_BAR_HOKUYO_X                     130*65536
#define PARAM_BAR_HOKUYO_Y                    -130*65536
#define PARAM_BAR_HOKUYO_ALPHA              (-1 << 23)
#define PARAM_BAR_HOKUYO_SENS                    1

#endif
