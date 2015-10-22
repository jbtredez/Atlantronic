#ifndef KINEMATICS_MODEL_DIFF_H
#define KINEMATICS_MODEL_DIFF_H

//! @file KinematicsModelDiff.h
//! @brief Kinematics model
//! @author Atlantronic

#include "KinematicsModel.h"

class KinematicsModelDiff : public KinematicsModel
{
	public:
		VectPlan computeSpeed(double voie, Kinematics* kinematics_mes);

		//!< calcul des consignes au niveau des moteurs avec saturations
		//!< @return coefficient multiplicateur applique sur speed pour respecter les saturations
		float computeActuatorCmd(double voie_inv, VectPlan u, float speed, float dt, Kinematics* kinematics_cmd);
};

#endif
