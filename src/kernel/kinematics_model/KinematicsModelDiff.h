#ifndef KINEMATICS_MODEL_DIFF_H
#define KINEMATICS_MODEL_DIFF_H

//! @file KinematicsModelDiff.h
//! @brief Kinematics model
//! @author Atlantronic

#include "KinematicsModel.h"

class KinematicsModelDiff : public KinematicsModel
{
	public:
		KinematicsModelDiff(float voie);

		VectPlan computeSpeed(Kinematics* kinematics_mes);

		//!< calcul des consignes au niveau des moteurs avec saturations
		//!< @return coefficient multiplicateur applique sur speed pour respecter les saturations
		float computeActuatorCmd(VectPlan u, float speed, float dt, Kinematics* kinematics_cmd);

	protected:
		float m_voie;
};

#endif