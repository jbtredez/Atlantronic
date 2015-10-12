#include "MotionActuatorKinematicsState.h"

MotionActuatorKinematicsState::MotionActuatorKinematicsState() :
	MotionMoveState("MOTION_ACTUATOR_KINEMATICS", MOTION_ACTUATOR_KINEMATICS)
{

}
void MotionActuatorKinematicsState::entry(void* data)
{
	Motion* m = (Motion*) data;
	//Satisfaction de la volonte operateur
	m->m_wantedState = MOTION_UNKNOWN_STATE;
}


void MotionActuatorKinematicsState::run(void* data)
{
	Motion* m = (Motion*) data;
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		if(m->m_wantedKinematics.mode[i] == KINEMATICS_SPEED)
		{
			m->m_kinematics[i].v = m->m_wantedKinematics.val[i];
			m->m_kinematics[i].mode = KINEMATICS_SPEED;
		}
		else if(m->m_wantedKinematics.mode[i] == KINEMATICS_POSITION)
		{
			m->m_kinematics[i].pos = m->m_wantedKinematics.val[i];
			m->m_kinematics[i].mode = KINEMATICS_POSITION;
		}
	}

	m->motionUpdateMotors();
}
