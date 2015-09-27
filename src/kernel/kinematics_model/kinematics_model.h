#ifndef KINEMATICS_MODEL_H
#define KINEMATICS_MODEL_H

//! @file kinematics_model.h
//! @brief Kinematics model
//! @author Atlantronic

#include <stdint.h>
#include "kernel/math/VectPlan.h"
#include "kernel/control/kinematics.h"

VectPlan kinematics_model_compute_speed(double voie, Kinematics* kinematics_mes);

//!< calcul des consignes au niveau des moteurs avec saturations
//!< @return coefficient multiplicateur applique sur speed pour respecter les saturations
float kinematics_model_compute_actuator_cmd(double voie_inv, VectPlan u, float speed, float dt, Kinematics* kinematics_cmd);


#endif
