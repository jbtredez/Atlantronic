//! @file kinematics_model_diff.cxx
//! @brief Kinematics model
//! @author Atlantronic

#include "KinematicsModelDiff.h"
#include "disco/robot_parameters.h"
#include "kernel/log.h"

#include <math.h>

KinematicsModelDiff::KinematicsModelDiff(float voie, KinematicsParameters paramDriving)
{
	m_voie = voie;
	m_paramDriving = paramDriving;
}

//!< calcul des consignes au niveau des moteurs avec saturations
//!< @return coefficient multiplicateur applique sur speed pour respecter les saturations
float KinematicsModelDiff::computeActuatorCmd(VectPlan u, float speed, float dt, Kinematics* kinematics_cmd)
{
	float kmin = 1;

	float vx = u.x * speed;
	float vtheta = u.theta * speed;
	float v[2];

	v[RIGHT_WHEEL] = vx + 0.5 * m_voie * vtheta;
	v[LEFT_WHEEL] = vx - 0.5 * m_voie * vtheta;

#if 1
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
#endif

	kinematics_cmd[0].setSpeed(kmin * v[0], m_paramDriving, dt);
	kinematics_cmd[1].setSpeed(kmin * v[1], m_paramDriving, dt);

	return kmin;
}

VectPlan KinematicsModelDiff::computeSpeed(Kinematics* kinematics_mes)
{
	VectPlan v;
	v.x = 0.5 * (kinematics_mes[RIGHT_WHEEL].v + kinematics_mes[LEFT_WHEEL].v);
	v.y = 0;
	v.theta = (kinematics_mes[RIGHT_WHEEL].v -  kinematics_mes[LEFT_WHEEL].v) / m_voie;

	return v;
}
