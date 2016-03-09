#include "MotionMoveState.h"
#include "kernel/driver/power.h"	

MotionMoveState::MotionMoveState(const char* name,unsigned int stateId)
	: StateMachineState(name, stateId)
{
}


unsigned int MotionMoveState::transition(void* data)
{
	Motion* m = (Motion*) data;

	bool all_op_enable = true;

	for(int i = 0; i < MOTION_MOTOR_MAX; i++)
	{
		all_op_enable &= m->m_motionMotor[i]->is_op_enable();
	}

	///En gros on veut éteindre les moteur cad passer dans l'etat DISABLE
	/// On par en Disable si pas de puissance dans les moteur ou on veut eteindre les moteur en partant en DISABLE
	if( power_get() || ! all_op_enable || m->m_wantedState == MOTION_DISABLED)
	{
		return MOTION_DISABLED;
	}

	return m_stateId;
}


