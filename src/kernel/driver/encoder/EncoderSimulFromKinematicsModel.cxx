#include "EncoderSimulFromKinematicsModel.h"

void EncoderSimulFromKinematicsModel::init(Location* location, KinematicsModelDiff* model, int wheelId)
{
	m_location = location;
	m_model = model;
	m_wheelId = wheelId;
}

void EncoderSimulFromKinematicsModel::update(float dt)
{
	VectPlan v = m_location->getSpeed();
	VectPlan u;
	float n = v.norm();
	if( n > EPSILON )
	{
		u = v / n;
	}
	else
	{
		n = v.theta;
		u = VectPlan(0, 0, 1);
	}
	m_model->computeActuatorCmd(u, n, dt, m_kinematics, false);
}

float EncoderSimulFromKinematicsModel::getPosition()
{
	return 0;
}

float EncoderSimulFromKinematicsModel::getSpeed()
{
	return m_kinematics[m_wheelId].v;
}
