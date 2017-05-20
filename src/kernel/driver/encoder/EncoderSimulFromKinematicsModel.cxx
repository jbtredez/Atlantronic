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
	m_model->computeActuatorCmd(v, 1, dt, m_kinematics, false);
}

float EncoderSimulFromKinematicsModel::getPosition()
{
	return 0;
}

float EncoderSimulFromKinematicsModel::getSpeed()
{
	return m_kinematics[m_wheelId].v;
}
