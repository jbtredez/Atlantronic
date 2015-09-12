/*
 * CStateMotionTryEnable.cpp
 *
 *  Created on: 23 août 2015
 *      Author: jul
 */
#include "kernel/log.h"
#include "CStateMotionTryEnable.h"

CStateMotionTryEnable::CStateMotionTryEnable():MotionEtat("MOTION_STATE_TRY_ENABLE")
{
	// TODO Auto-generated constructor stub
	m_motion_State 				= MOTION_STATE_TRY_ENABLE;

}

CStateMotionTryEnable::~CStateMotionTryEnable()
{
	// TODO Auto-generated destructor stub
}


////////////////////////////////////////
//méthode virtuelle Effectue l'action de l'etat
//Param :
//retourne: Réussite de l'action
bool CStateMotionTryEnable::run()
{
	log_format(LOG_INFO, "Run dans l'etat %s", this->getNameEtat());
	//On étient les moteurs
	bool op_enable = true;
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		m_motion_kinematics[i].pos = m_motion_kinematics_mes[i].pos;
		m_motion_kinematics[i].v = 0;
		can_motor[i].enable(true);
		op_enable &= (!can_motor[i].is_op_enable());
	}
	return op_enable;
}

////////////////////////////////////////
//méthode virtuelle Effectue l'action de l'etat
//Param :
//retourne: Réussite de l'action
bool CStateMotionTryEnable::entry()
{

	log_format(LOG_INFO, "Entree dans l'etat %s", this->getNameEtat());

	return true;
}

////////////////////////////////////////
//méthode virtuelle Effectue l'action de l'etat
//Param :
//retourne: Réussite de l'action
bool CStateMotionTryEnable::out()
{

	log_format(LOG_INFO, "Sortie de l'etat %s", this->getNameEtat());

	return true;
}



////////////////////////////////////////
//méthode recupere l'etat suivant
//Param :
//retourne: Id de l'etat suivant
Etat * CStateMotionTryEnable::getProchainEtat()
{
	Etat * pFuturState = this;

	bool all_op_enable = true;

	if( power_get() || m_motion_Wanted_State == MOTION_STATE_DISABLED )
	{
		// puissance desactivee
		pFuturState = m_pMotionDisable;
	}

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		all_op_enable &= can_motor[i].is_op_enable();
	}

	if( all_op_enable )
	{
		// tout les moteurs sont en op_enable
		pFuturState = m_pMotionEnable;
	}

	return pFuturState;

}
