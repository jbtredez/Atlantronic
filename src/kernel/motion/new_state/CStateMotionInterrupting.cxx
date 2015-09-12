
#include "kernel/log.h"
#include "kernel/motion/new_state/MotionVar.h"
#include "kernel/motion/new_state/CMotionEtat.h"
#include "kernel/motion/new_state/CStateMotionDisable.h"



CStateMotionInterrupting::CStateMotionInterrupting():MotionEtat("MOTION_STATE_INTERRUPTING")
{
	m_pMotionDisable = 0;
	m_pMotionEnable = 0;
	m_motion_State 				= MOTION_STATE_INTERRUPTING;
}
CStateMotionInterrupting::~CStateMotionInterrupting()
{
}



////////////////////////////////////////
//méthode virtuelle Effectue l'action de l'etat
//Param :
//retourne: Réussite de l'action		
bool CStateMotionInterrupting::run()
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
//méthode virtuelle Effectue l'action de l'etat
//Param :
//retourne: Réussite de l'action		
bool CStateMotionInterrupting::entry()
{


	log_format(LOG_INFO, "Entree dans l'etat %s", this->getNameEtat());

	//Prise en compte de la commande utilisateur de changement d'état
	m_motion_Wanted_State = MOTION_NONE_STATE;
	return true;
}
		
////////////////////////////////////////
//méthode virtuelle Effectue l'action de l'etat
//Param :
//retourne: Réussite de l'action		
bool CStateMotionInterrupting::out()
{

	log_format(LOG_INFO, "Sortie de l'etat %s", this->getNameEtat());

	return true;
}



////////////////////////////////////////
//méthode recupere l'etat suivant
//Param :
//retourne: Id de l'etat suivant		
Etat * CStateMotionInterrupting::getProchainEtat()
{
	Etat * pFuturState = this;
	bool all_op_enable = true;

	for(int i = 0; i < CAN_MOTOR_MAX; i++ )
	{
		if( can_motor[i].is_in_motion() )
		{
			return pFuturState;
		}
	}

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
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
