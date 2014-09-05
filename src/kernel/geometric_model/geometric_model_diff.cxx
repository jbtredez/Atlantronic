//! @file geometric_model.cxx
//! @brief Geometric model
//! @author Atlantronic

#include "geometric_model.h"
#include "kernel/robot_parameters.h"
#include "kernel/log.h"

#include <math.h>

#define EPSILON                                 1e-4

static KinematicsParameters paramDriving = {1500, 1500, 1500};

#define VOIE                       200
#define VOIE_INVERSE             0.005 //  = 1/200

//!< calcul des consignes au niveau des moteurs avec saturations
//!< @return coefficient multiplicateur applique sur speed pour respecter les saturations
float geometric_model_compute_actuator_cmd(VectPlan /*cp*/, VectPlan u, float speed, float dt, Kinematics* kinematics_cmd, int* wheelReady)
{
	float kmin = 1;

	// pas de rotation de roue
	*wheelReady = 1;
	float vx = u.x * speed;
	float vtheta = u.theta * speed;
	float v[2];

	v[0] = vx - 0.5 * VOIE * vtheta;
	v[1] = vx + 0.5 * VOIE * vtheta;

	int i;
	for(i = 0; i < 2; i++)
	{
		Kinematics kinematics = kinematics_cmd[i];
		kinematics.setSpeed(v[i], paramDriving, dt);

		// reduction si saturation
		float k = fabsf(kinematics.v / v[i]);
		if( k < kmin )
		{
			kmin = k;
		}
	}

	kinematics_cmd[0].setSpeed(v[0], paramDriving, dt);
	kinematics_cmd[1].setSpeed(v[1], paramDriving, dt);

	return kmin;
}

VectPlan geometric_model_compute_speed(Kinematics* kinematics_mes, float* slippageSpeed)
{
	VectPlan v;
	v.x = 0.5 * (kinematics_mes[0].v + kinematics_mes[1].v);
	v.y = 0;
	v.theta = VOIE_INVERSE * (kinematics_mes[1].v -  kinematics_mes[0].v);

	if( slippageSpeed )
	{
		*slippageSpeed = 0;
	}

	return v;
}
