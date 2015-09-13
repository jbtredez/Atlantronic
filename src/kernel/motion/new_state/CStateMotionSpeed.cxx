/*
 * CMotionSpeed.cpp
 *
 *  Created on: 24 août 2015
 *      Author: jul
 */
#include "kernel/log.h"
#include "kernel/kinematics_model/kinematics_model.h"
#include "kernel/motion/new_state/CMotionEtat.h"
#include "CStateMotionSpeed.h"


CStateMotionSpeed::CStateMotionSpeed():MotionEtat("MOTION_STATE_SPEED")
{
	// TODO Auto-generated constructor stub
	m_motion_State 		= MOTION_STATE_SPEED;
	m_pMotionDisable	= 0;
	m_pMotionEnable		= 0;

}

CStateMotionSpeed::~CStateMotionSpeed()
{
	// TODO Auto-generated destructor stub
}


////////////////////////////////////////
//méthode virtuelle Effectue l'action de l'etat
//Param :
//retourne: Réussite de l'action
bool CStateMotionSpeed::run()
{
	kinematics_model_compute_actuator_cmd(VOIE_MOT, m_motion_u, m_motion_v, CONTROL_DT, m_motion_kinematics);
	motion_update_motors();
	return true;
}

////////////////////////////////////////
//méthode virtuelle Effectue l'action de l'etat
//Param :
//retourne: Réussite de l'action
bool CStateMotionSpeed::entry()
{

	log_format(LOG_INFO, "Entree dans l'etat %s", this->getNameEtat());
	m_motion_status = MOTION_IN_MOTION;

	log(LOG_INFO, "IN_MOTION SPEED");

	//Prise en compte de la commande utilisateur de changement d'état
	m_motion_Wanted_State = MOTION_NONE_STATE;
	return true;
}


////////////////////////////////////////
//méthode recupere l'etat suivant
//Param :
//retourne: Id de l'etat suivant
Etat * CStateMotionSpeed::getProchainEtat()
{
	Etat * pFuturState = this;
	bool all_op_enable = true;

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		all_op_enable &= can_motor[i].is_op_enable();
	}

	if( power_get() || ! all_op_enable || m_motion_Wanted_State == MOTION_STATE_DISABLED)
	{
		pFuturState = m_pMotionDisable;
	}
	else if( m_motion_Wanted_State == MOTION_STATE_ENABLED)
	{
		pFuturState = m_pMotionEnable;
	}


	return pFuturState;
}
