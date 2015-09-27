#include "MotionDisabledState.h"
#include "kernel/driver/power.h"

MotionDisabledState::MotionDisabledState() :
	StateMachineState("MOTION_DISABLED")
{

}

void MotionDisabledState::entry(void* data)
{
	Motion* m = (Motion*) data;
#ifndef MOTION_AUTO_ENABLE
	m->m_enableWanted = MOTION_ENABLE_WANTED_UNKNOWN;
#else
	m->m_enableWanted = MOTION_ENABLE_WANTED_ON;
#endif
}

void MotionDisabledState::run(void* data)
{
	Motion* m = (Motion*) data;
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		m->m_kinematics[i].pos = m->m_kinematicsMes[i].pos;
		m->m_kinematics[i].v = 0;
		m->m_canMotor[i].enable(false);
	}
}

unsigned int MotionDisabledState::transition(void* data, unsigned int currentState)
{
	Motion* m = (Motion*) data;
#ifndef MOTION_AUTO_ENABLE
	if( power_get() )
	{
		// puissance desactivee
		m->m_enableWanted = MOTION_ENABLE_WANTED_UNKNOWN;
	}
#else
	if( ! power_get() )
	{
		m->m_enableWanted = MOTION_ENABLE_WANTED_ON;
	}
#endif

	if( m->m_enableWanted == MOTION_ENABLE_WANTED_ON && ! power_get() )
	{
		return MOTION_TRY_ENABLE;
	}

	return currentState;
}
