#ifndef ODOMETRY_H
#define ODOMETRY_H

//! @file odometry.h
//! @brief Odometry
//! @author Atlantronic

#include <stdint.h>
#include "kernel/math/vect_plan.h"
#include "kernel/control/kinematics.h"

VectPlan geometric_model_compute_speed(Kinematics* kinematics_mes, float* slippageSpeed);

//!< calcul des consignes au niveau des moteurs avec saturations
//!< @return coefficient multiplicateur applique sur speed pour respecter les saturations
float geometric_model_compute_actuator_cmd(VectPlan cp, VectPlan u, float speed, float dt, Kinematics* kinematics_cmd);


#endif
