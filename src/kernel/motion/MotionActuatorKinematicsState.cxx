#include "MotionActuatorKinematicsState.h"

MotionActuatorKinematicsState::MotionActuatorKinematicsState() :
	StateMachineState("MOTION_ACTUATOR_KINEMATICS")
{

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

	m->motion_update_motors();
}

unsigned int MotionActuatorKinematicsState::transition(void* data, unsigned int currentState)
{
	Motion* m = (Motion*) data;
	return m->motion_state_generic_power_transition(currentState);
}
