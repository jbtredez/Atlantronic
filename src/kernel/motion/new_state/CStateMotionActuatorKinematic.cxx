/*
 * CStateMotionActuatorKinematic.cxx
 *
 *  Created on: 24 août 2015
 *      Author: jul
 */
#include "kernel/log.h"
#include "kernel/motion/new_state/CMotionEtat.h"
#include "CStateMotionActuatorKinematic.h"

CStateMotionActuatorKinematic::CStateMotionActuatorKinematic():MotionEtat("MOTION_STATE_ACTUATOR_KINEMATICS")
{
	// TODO Auto-generated constructor stub
	m_motion_State 				= MOTION_STATE_ACTUATOR_KINEMATICS;

}

CStateMotionActuatorKinematic::~CStateMotionActuatorKinematic()
{
	// TODO Auto-generated destructor stub
}


////////////////////////////////////////
//méthode virtuelle Effectue l'action de l'etat
//Param :
//retourne: Réussite de l'action
bool CStateMotionActuatorKinematic::run()
{
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		if(m_pmotion_wanted_kinematics->mode[i] == KINEMATICS_SPEED)
		{
			m_motion_kinematics[i].v = m_pmotion_wanted_kinematics->val[i];
			m_motion_kinematics[i].mode = KINEMATICS_SPEED;
		}
		else if(m_pmotion_wanted_kinematics->mode[i] == KINEMATICS_POSITION)
		{
			m_motion_kinematics[i].pos = m_pmotion_wanted_kinematics->val[i];
			m_motion_kinematics[i].mode = KINEMATICS_POSITION;
		}
	}

	motion_update_motors();
	return true;
}



////////////////////////////////////////
//méthode virtuelle Effectue l'action de l'etat
//Param :
//retourne: Réussite de l'action
bool CStateMotionActuatorKinematic::entry()
{
//#ifndef MOTION_AUTO_ENABLE
//	motion_enable_wanted = MOTION_ENABLE_WANTED_UNKNOWN;
//#else
//	motion_enable_wanted = MOTION_ENABLE_WANTED_ON;
//#endif

	log_format(LOG_INFO, "Entree dans l'etat %s", this->getNameEtat());

	//Prise en compte de la commande utilisateur de changement d'état
	m_motion_Wanted_State = MOTION_NONE_STATE;
	return true;
}

////////////////////////////////////////
//méthode virtuelle Effectue l'action de l'etat
//Param :
//retourne: Réussite de l'action
bool CStateMotionActuatorKinematic::out()
{

	log_format(LOG_INFO, "Sortie de l'etat %s", this->getNameEtat());

	return true;
}



////////////////////////////////////////
//méthode recupere l'etat suivant
//Param :
//retourne: Id de l'etat suivant
Etat * CStateMotionActuatorKinematic::getProchainEtat()
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

void CStateMotionActuatorKinematic::motion_set_actuator_kinematics(motion_cmd_set_actuator_kinematics_arg cmd)
{
	xSemaphoreTake(m_motion_mutex, portMAX_DELAY);

	m_motion_Wanted_State = MOTION_STATE_ACTUATOR_KINEMATICS;
	*m_pmotion_wanted_kinematics = cmd;

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		if( cmd.mode[i] != KINEMATICS_POSITION && cmd.mode[i] != KINEMATICS_SPEED)
		{
			log_format(LOG_ERROR, "unknown mode %d for actuator %i", cmd.mode[i], i);
			m_motion_Wanted_State = MOTION_NONE_STATE;
		}
	}

	xSemaphoreGive(m_motion_mutex);
}
