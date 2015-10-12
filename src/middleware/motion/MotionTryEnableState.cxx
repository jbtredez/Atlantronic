#include "MotionTryEnableState.h"

#include "kernel/driver/power.h"

MotionTryEnableState::MotionTryEnableState() :
	StateMachineState("MOTION_TRY_ENABLE",MOTION_TRY_ENABLE)
{

}

void MotionTryEnableState::run(void* data)
{
	Motion* m = (Motion*) data;
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		m->m_kinematics[i].pos = m->m_kinematicsMes[i].pos;
		m->m_kinematics[i].v = 0;
		m->m_canMotor[i].enable(true);
	}
}

unsigned int MotionTryEnableState::transition(void* data)
{
	Motion* m = (Motion*) data;
	bool all_op_enable = true;

	//Volonte de l'utilisateur de couper l'etat ou pas de puissance
	if( power_get() || m->m_wantedState == MOTION_DISABLED )
	{
		// puissance desactivee
		return MOTION_DISABLED;
	}

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		all_op_enable &= m->m_canMotor[i].is_op_enable();
	}

	if( all_op_enable )
	{
		// tout les moteurs sont en op_enable
		return MOTION_ENABLED;
	}

	return m_stateId;
}
