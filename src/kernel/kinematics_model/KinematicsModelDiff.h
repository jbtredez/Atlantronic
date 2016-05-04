#ifndef KINEMATICS_MODEL_DIFF_H
#define KINEMATICS_MODEL_DIFF_H

//! @file KinematicsModelDiff.h
//! @brief Kinematics model
//! @author Atlantronic

#include "KinematicsModel.h"

class KinematicsModelDiff : public KinematicsModel
{
	public:
		KinematicsModelDiff(float voieSensPositif, float voieSensNegatif, KinematicsParameters paramDriving);

		VectPlan computeSpeed(Kinematics* kinematics_mes);

		//!< calcul des consignes au niveau des moteurs avec saturations
		//!< @return coefficient multiplicateur applique sur speed pour respecter les saturations
		float computeActuatorCmd(VectPlan u, float speed, float dt, Kinematics* kinematics_cmd, bool saturate);
		void  setOdoVoie(float odovoie){m_voieSensPositif = odovoie;}; // TODO
		float getOdoVoie(){return m_voieSensPositif;}; // TODO

	protected:
		float m_voieSensPositif;
		float m_voieSensNegatif;
		KinematicsParameters m_paramDriving;
};

#endif
