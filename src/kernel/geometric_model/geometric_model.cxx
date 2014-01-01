//! @file geometric_model.cxx
//! @brief Geometric model
//! @author Atlantronic

#include "geometric_model.h"
#include <math.h>

#define EPSILON                                 1e-4

static KinematicsParameters paramDriving = {1500, 1000, 1500};
static KinematicsParameters paramSteering = {1.5, 1.5, 1.5};

// TODO turret[3] en doublon avec control.cxx
static VectPlan Turret[3] =
{
	VectPlan(   0,  155, 0),
	VectPlan(   0, -155, 0),
	VectPlan(-175,    0, 0)
};

//!< calcul des consignes au niveau des moteurs avec saturations
//!< @return coefficient multiplicateur applique sur speed pour respecter les saturations
float geometric_model_compute_actuator_cmd(VectPlan cp, VectPlan u, float speed, float dt, Kinematics* kinematics_cmd)
{
	float kmin = 1;
	float theta[3];
	float v[3];
	float w[3];

	// saturation liee a la traction
	for(int i = 0; i < 3; i++)
	{
		VectPlan vOnTurret = transferSpeed(cp, Turret[i], u);
		float n2 = vOnTurret.norm2();
		v[i] = speed * sqrtf(n2);
		theta[i] = atan2f(vOnTurret.y, vOnTurret.x);
		float theta_old = kinematics_cmd[2*i+1].pos;

		// on minimise la rotation des roues
		float dtheta1 = fmodf(theta[i] - theta_old, 2*M_PI);
		if( dtheta1 > M_PI)
		{
			dtheta1 -= 2*M_PI;
		}
		else if(dtheta1 < -M_PI)
		{
			dtheta1 += 2*M_PI;
		}

		float dtheta2 = fmodf(theta[i] + M_PI - theta_old, 2*M_PI);
		if( dtheta2 > M_PI)
		{
			dtheta2 -= 2*M_PI;
		}
		else if(dtheta2 < -M_PI)
		{
			dtheta2 += 2*M_PI;
		}

		if( fabsf(dtheta1) < fabsf(dtheta2) )
		{
			theta[i] = theta_old + dtheta1;
		}
		else
		{
			theta[i] = theta_old + dtheta2;
			v[i] *= -1;
		}

		// TODO couplage traction direction
		//v[i] += rayonRoue * k * w[i]; avec k = 0.25

		Kinematics kinematics = kinematics_cmd[2*i];
		kinematics.setSpeed(v[i], paramDriving, dt);
		if( fabsf(v[i]) > EPSILON)
		{
			float k = fabsf(kinematics.v / v[i]);
			if( k < kmin )
			{
				kmin = k;
			}
		}

		w[i] = - u.theta * speed * (u.x * vOnTurret.x + vOnTurret.y * u.y) / n2;
		// TODO gestion saturation rotation tourelle ko
		/*kinematics = control_kinematics[i+3];
		kinematics.setPosition(theta[i], w[i], paramSteering, dt);
		if( fabsf(w[i]) > EPSILON)
		{
			float k = fabsf(kinematics.v / w[i]);
			if( k < kmin )
			{
				kmin = k;
			}
		}*/
	}

	for(int i = 0; i < 3; i++)
	{
		kinematics_cmd[2*i].setSpeed(v[i] * kmin, paramDriving, dt);
		kinematics_cmd[2*i+1].setPosition(theta[i], w[i] * kmin, paramSteering, dt);
	}
	//log_format(LOG_INFO, "v %d %d %d", (int)(1000*control_kinematics[0].v), (int)(1000*control_kinematics[1].v), (int)(1000*control_kinematics[2].v));

	return kmin;
}

static VectPlan odometry2turret(const VectPlan &cp, const VectPlan &A, const VectPlan &B, const VectPlan &v1, const VectPlan &v2, float* slippageSpeed)
{
	VectPlan res(0,0,0);
	float dx = B.x - A.x;
	float dy = B.y - A.y;
	float dv = 0;

	// on divise par le plus grand pour eviter les pb numeriques
	if( fabsf(dx) > fabsf(dy) )
	{
		res.theta = (v2.y - v1.y) / dx;
		dv = fabsf(v1.x - v2.x - dy * res.theta);
	}
	else if( fabsf(dy) > 0 )
	{
		res.theta = (v1.x - v2.x) / dy;
		dv = fabsf(v2.y - v1.y - dx * res.theta);
	}
	else
	{
		// calcul non realisable, A et B sont au même endroit
		// on retourne une vitesse nulle
		goto end;
	}

	res.x = 0.5 * (v1.x + v2.x + res.theta * (A.y + B.y - 2 * cp.y));
	res.y = 0.5 * (v1.y + v2.y + res.theta * ( 2 * cp.x - A.x - B.x));

end:
	if( slippageSpeed )
	{
		*slippageSpeed = dv;
	}

	return res;
}

VectPlan geometric_model_compute_speed(Kinematics* kinematics_mes, float* slippageSpeed)
{
	VectPlan v[3];
	for(int i = 0; i < 3; i++)
	{
		float phi = kinematics_mes[2*i+1].pos;
		v[i].x = kinematics_mes[2*i].v * cosf(phi);
		v[i].y = kinematics_mes[2*i].v * sinf(phi);
		//log_format(LOG_INFO, "%d %d %d %d", i, (int)(1000*v[i].x), (int)(1000*v[i].y), (int)(phi * 180 / M_PI));
	}

	// TODO odometrie sur 2 tourelles uniquement
	return odometry2turret(VectPlan(0,0,0), Turret[0], Turret[1], v[0], v[1], slippageSpeed);
}
