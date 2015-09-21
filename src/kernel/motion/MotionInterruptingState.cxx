#include "MotionInterruptingState.h"

MotionInterruptingState::MotionInterruptingState() :
	StateMachineState("MOTION_INTERRUPTING")
{

}

void MotionInterruptingState::run(void* data)
{
	Motion* m = (Motion*) data;
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		m->m_kinematics[i].v = 0;
	}
	m->motion_update_motors();
}

unsigned int MotionInterruptingState::transition(void* data, unsigned int currentState)
{
	Motion* m = (Motion*) data;
	for(int i = 0; i < CAN_MOTOR_MAX; i++ )
	{
		if( can_motor[i].is_in_motion() )
		{
			return MOTION_INTERRUPTING;
		}
	}

	unsigned int newState = m->motion_state_generic_power_transition(currentState);
	if( newState != currentState )
	{
		return newState;
	}

	return MOTION_ENABLED;
}
