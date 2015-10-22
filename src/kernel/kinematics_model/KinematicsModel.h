#ifndef KINEMATICS_MODEL_H
#define KINEMATICS_MODEL_H

//! @file kinematics_model.h
//! @brief Kinematics model
//! @author Atlantronic

#include <stdint.h>
#include "kernel/math/VectPlan.h"
#include "kernel/control/kinematics.h"

class KinematicsModel
{
	public:
		virtual VectPlan computeSpeed(double voie, Kinematics* kinematics_mes) = 0;

		//!< calcul des consignes au niveau des moteurs avec saturations
		//!< @return coefficient multiplicateur applique sur speed pour respecter les saturations
		virtual float computeActuatorCmd(double voie_inv, VectPlan u, float speed, float dt, Kinematics* kinematics_cmd) = 0;
};

#endif
