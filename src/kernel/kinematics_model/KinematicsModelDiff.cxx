//! @file kinematics_model_diff.cxx
//! @brief Kinematics model
//! @author Atlantronic

#include "KinematicsModelDiff.h"
#include "kernel/log.h"
#include "disco/bot.h"

#include <math.h>

KinematicsModelDiff::KinematicsModelDiff(float voie, KinematicsParameters paramDriving)
{
	m_voie = voie;
	m_paramDriving = paramDriving;
}

//!< calcul des consignes au niveau des moteurs avec saturations
//!< @return coefficient multiplicateur applique sur speed pour respecter les saturations
float KinematicsModelDiff::computeActuatorCmd(VectPlan u, float speed, float dt, Kinematics* kinematics_cmd, bool saturate)
{
	float kmin = 1;

	float vx = u.x * speed;
	float vtheta = u.theta * speed;
	float v[2];

	v[Bot::rightWheel] = vx + 0.5 * m_voie * vtheta;
	v[Bot::leftWheel] = vx - 0.5 * m_voie * vtheta;

	if( saturate )
	{
		// TODO voir si ca marche bien
		for(int i = 0; i < 2; i++)
		{
			Kinematics kinematics = kinematics_cmd[i];
			kinematics.setSpeed(v[i], m_paramDriving, dt);

			// reduction si saturation
			if( fabsf(v[i]) > 1 )
			{
				float k = fabsf(kinematics.v / v[i]);
				if( k < kmin )
				{
					kmin = k;
				}
			}
		}
		kinematics_cmd[0].setSpeed(kmin * v[0], m_paramDriving, dt);
		kinematics_cmd[1].setSpeed(kmin * v[1], m_paramDriving, dt);
	}
	else
	{
		kinematics_cmd[0].v = v[0];
		kinematics_cmd[1].v = v[1];
	}

	return kmin;
}

VectPlan KinematicsModelDiff::computeSpeed(Kinematics* kinematics_mes)
{
	VectPlan v;
	v.x = 0.5 * (kinematics_mes[Bot::rightWheel].v + kinematics_mes[Bot::leftWheel].v);
	v.y = 0;
	v.theta = (kinematics_mes[Bot::rightWheel].v -  kinematics_mes[Bot::leftWheel].v) / m_voie;

	return v;
}
