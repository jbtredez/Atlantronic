
#include "kernel/log.h"
#include "kernel/motion/new_state/CMotionEtat.h"
#include "kernel/motion/new_state/CStateMotionDisable.h"



CStateMotionDisable::CStateMotionDisable():MotionEtat("MOTION_STATE_DISABLE")
{
	m_pMotionTryEnable 	= 0;
	m_motion_State		= MOTION_STATE_DISABLED;
}
CStateMotionDisable::~CStateMotionDisable()
{
}



////////////////////////////////////////
//méthode virtuelle Effectue l'action de l'etat
//Param :
//retourne: Réussite de l'action		
bool CStateMotionDisable::run()
{
	//On étient les moteurs
	bool op_enable = true;
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		m_motion_kinematics[i].pos = m_motion_kinematics_mes[i].pos;
		m_motion_kinematics[i].v = 0;
		can_motor[i].enable(false);
		op_enable &= (!can_motor[i].is_op_enable());
	}
	return op_enable;
}		
#ifdef MOTION_AUTO_ENABLE
////////////////////////////////////////
//méthode virtuelle Effectue l'action de l'etat
//Param :
//retourne: Réussite de l'action		
bool CStateMotionDisable::entry()
{
	m_motion_Wanted_State = MOTION_STATE_ENABLED;

	log_format(LOG_INFO, "Entree dans l'etat %s", this->getNameEtat());

	//Prise en compte de la commande utilisateur de changement d'état
	return true;
}
		
#endif

////////////////////////////////////////
//méthode recupere l'etat suivant
//Param :
//retourne: Id de l'etat suivant		
Etat * CStateMotionDisable::getProchainEtat()
{
	Etat * pFuturState = this;

	//On quitte l'etat que si et seulement si on veut MOTION_TRY_ENABLE && que l'alimentation fonctionne
	if( (MOTION_STATE_TRY_ENABLE == m_motion_Wanted_State  || MOTION_STATE_ENABLED == m_motion_Wanted_State  ) && ! power_get() )
	{
		//return MOTION_TRY_ENABLE;
		pFuturState = m_pMotionTryEnable;
	}

	return pFuturState;
}
