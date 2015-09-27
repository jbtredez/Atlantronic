#include "MotionEnabledState.h"

#include "kernel/driver/power.h"

MotionEnabledState::MotionEnabledState() :
	StateMachineState("MOTION_ENABLED")
{

}

void MotionEnabledState::entry(void* data)
{
	Motion* m = (Motion*) data;
	if( m->m_enableWanted == MOTION_ENABLE_WANTED_ON )
	{
#ifndef MOTION_AUTO_ENABLE
		m->m_enableWanted = MOTION_ENABLE_WANTED_UNKNOWN;
#endif
	}
	m->m_wantedState = MOTION_WANTED_STATE_UNKNOWN;
}

void MotionEnabledState::run(void* data)
{
	Motion* m = (Motion*) data;
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		m->m_kinematics[i].v = 0;
		m->m_kinematics[i].mode = KINEMATICS_SPEED;
	}

	m->motionUpdateMotors();
}

unsigned int MotionEnabledState::transition(void* data, unsigned int currentState)
{
	Motion* m = (Motion*) data;
	bool all_op_enable = true;

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		all_op_enable &= m->m_canMotor[i].is_op_enable();
	}

	if( power_get() || ! all_op_enable || m->m_enableWanted == MOTION_ENABLE_WANTED_OFF )
	{
		return MOTION_DISABLED;
	}

	switch(m->m_wantedState)
	{
		case MOTION_WANTED_STATE_ACTUATOR_KINEMATICS:
			return MOTION_ACTUATOR_KINEMATICS;
			break;
		case MOTION_WANTED_STATE_TRAJECTORY:
			return MOTION_TRAJECTORY;
			break;
		case MOTION_WANTED_STATE_SPEED:
			return MOTION_SPEED;
		case MOTION_WANTED_STATE_UNKNOWN:
		default:
			break;
	}

	return currentState;
}
