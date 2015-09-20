
#include "kernel/log.h"
#include "kernel/motion/new_state/MotionVar.h"
#include "MotionEtat.h"
#include "StateMotionDisable.h"



StateMotionInterrupting::StateMotionInterrupting():MotionEtat("MOTION_STATE_INTERRUPTING")
{
	m_pMotionDisable 	= 0;
	m_pMotionEnable 	= 0;
	m_motion_State 		= MOTION_STATE_INTERRUPTING;
}
StateMotionInterrupting::~StateMotionInterrupting()
{
}



////////////////////////////////////////
//méthode virtuelle Effectue l'action de l'etat
//Param :
//retourne: Réussite de l'action		
bool StateMotionInterrupting::run()
{
	//On stop les moteurs
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		m_motion_kinematics[i].v = 0;
	}
	motion_update_motors();
	return true;
}		




////////////////////////////////////////
//méthode recupere l'etat suivant
//Param :
//retourne: Id de l'etat suivant		
Etat * StateMotionInterrupting::getProchainEtat()
{
	Etat * pFuturState = this;
	bool all_op_enable = true;

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		if( can_motor[i].is_in_motion() )
		{
			return pFuturState;
		}

		all_op_enable &= can_motor[i].is_op_enable();
	}

	//On quitte l'etat que si et seulement si on veut MOTION_TRY_ENABLE && que l'alimentation fonctionne
	//Passage à l'état Disable à la fin du match ou demande utilisateur
	if( power_get() || ! all_op_enable || m_motion_Wanted_State == MOTION_STATE_DISABLED)
	{
		pFuturState = m_pMotionDisable;
	}

	//Passage à l'état Enable
	pFuturState = m_pMotionEnable;
	
	return pFuturState;
}
