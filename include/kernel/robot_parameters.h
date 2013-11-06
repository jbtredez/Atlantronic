#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H

//! @file robot_parameters.h
//! @brief Parameters
//! @author Atlantronic

#include <math.h>

#define PI_FX29                           1686629713      //!< pi en 2^-29 rd

#define PARAM_RIGHT_ODO_WHEEL_RADIUS_FX      2588574      //!< rayon de la roue droite en 2^-16 mm (virgule fixe) ( = 39.5 mm)
#define PARAM_LEFT_ODO_WHEEL_RADIUS_FX       2588574      //!< rayon de la roue droite en 2^-16 mm (virgule fixe) ( = 49.5 mm)

#define PARAM_INVERTED_VOIE_FX39          1782028570      //!< inverse de la voie en 2^-39 mm^-1 ( = 1 / 308.5mm)

#define PARAM_RIGHT_ODO_WHEEL_WAY                 1
#define PARAM_LEFT_ODO_WHEEL_WAY                  1
#define PARAM_RIGHT_MOT_WHEEL_WAY                 1
#define PARAM_LEFT_MOT_WHEEL_WAY                  1

#define PARAM_ENCODERS_BIT_RES                   12
#define PARAM_ODO_RED                             1

#define PARAM_LEFT_CORNER_X                    190
#define PARAM_LEFT_CORNER_Y                    175
#define PARAM_RIGHT_CORNER_X                   190
#define PARAM_RIGHT_CORNER_Y                  -175

#define PARAM_NP_X                            -63

#define PARAM_FOO_HOKUYO_X                     105
#define PARAM_FOO_HOKUYO_Y                     140
#define PARAM_FOO_HOKUYO_ALPHA                 (M_PI/4)
#define PARAM_FOO_HOKUYO_SENS                          1

#define PARAM_BAR_HOKUYO_X                     105
#define PARAM_BAR_HOKUYO_Y                    -140
#define PARAM_BAR_HOKUYO_ALPHA                (-M_PI/4)
#define PARAM_BAR_HOKUYO_SENS                          1

#endif
