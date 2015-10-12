#include "MotionInterruptingState.h"

MotionInterruptingState::MotionInterruptingState() :
	MotionMoveState("MOTION_INTERRUPTING",MOTION_INTERRUPTING)
{

}

void MotionInterruptingState::run(void* data)
{
	Motion* m = (Motion*) data;
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		m->m_kinematics[i].v = 0;
	}
	m->motionUpdateMotors();
}

unsigned int MotionInterruptingState::transition(void* data)
{
	Motion* m = (Motion*) data;
	
	for(int i = 0; i < CAN_MOTOR_MAX; i++ )
	{
		if( m->m_canMotor[i].is_in_motion() )
		{
			return MOTION_INTERRUPTING;
		}
	}
	
	unsigned int newState = MotionMoveState::transition(data);
	if( newState == MOTION_DISABLED )
	{
		return newState;
	}

	return MOTION_ENABLED;
}
