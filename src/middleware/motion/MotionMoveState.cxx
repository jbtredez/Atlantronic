#include "MotionMoveState.h"
#include "kernel/driver/power.h"	

MotionMoveState::MotionMoveState(const char* name,unsigned int stateId):StateMachineState(name,stateId)
{
}


unsigned int MotionMoveState::transition(void* data)
{

	Motion* m = (Motion*) data;

	bool all_op_enable = true;

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		all_op_enable &= m->m_canMotor[i].is_op_enable();
	}
	///En gros on veut éteindre les moteur cad passer dans l'etat DISABLE
	/// On par en DIsable si pas de puissance dans les moteur ou on veut eteindre les moteur en partant en DISABLE
	if( power_get() || ! all_op_enable || m->m_wantedState == MOTION_DISABLED)
	{
		return MOTION_DISABLED;
	}

	//On rentre dans ce If que dans l'état DISABLE ou TRY_ENABLE ou MOTION_ENABLE
	//Or cette méthode est appellée seulement par les ETATs autres que ces trois derniers....
	//Donc Code mort à supprimer
//	if( m_wantedState == MOTION_ENABLED && currentState != MOTION_ACTUATOR_KINEMATICS &&
//		currentState != MOTION_SPEED && currentState != MOTION_TRAJECTORY && currentState != MOTION_INTERRUPTING)
//	{
//		return MOTION_ENABLED;
//	}

	return m_stateId;
}


