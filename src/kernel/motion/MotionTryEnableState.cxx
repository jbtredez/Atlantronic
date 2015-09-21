#include "MotionTryEnableState.h"

#include "kernel/driver/power.h"

MotionTryEnableState::MotionTryEnableState() :
	StateMachineState("MOTION_TRY_ENABLE")
{

}

void MotionTryEnableState::run(void* data)
{
	Motion* m = (Motion*) data;
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		m->m_kinematics[i].pos = m->m_kinematicsMes[i].pos;
		m->m_kinematics[i].v = 0;
		can_motor[i].enable(true);
	}
}

unsigned int MotionTryEnableState::transition(void* data, unsigned int currentState)
{
	Motion* m = (Motion*) data;
	bool all_op_enable = true;

	if( power_get() || m->m_enableWanted == MOTION_ENABLE_WANTED_OFF )
	{
		// puissance desactivee
		return MOTION_DISABLED;
	}

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		all_op_enable &= can_motor[i].is_op_enable();
	}

	if( all_op_enable )
	{
		// tout les moteurs sont en op_enable
		return MOTION_ENABLED;
	}

	return currentState;
}
