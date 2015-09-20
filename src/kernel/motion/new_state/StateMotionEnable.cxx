
#include "kernel/log.h"
#include "MotionEtat.h"
#include "StateMotionEnable.h"


StateMotionEnable::StateMotionEnable():MotionEtat("MOTION_STATE_ENABLE")
{
	m_pMotionDisable 		= 0;
	m_pMotionActuorKinematics 	= 0;
	m_pMotionSpeed 			= 0;
	m_pMotionTrajectory		= 0;
	m_pmotion_wanted_kinematics 	= 0;
	m_pgotoparam			= 0;

}

void StateMotionEnable::initState(Etat * pMotionTrajectory, Etat * pMotionSpeed, Etat * pMotionActuorKinematics, Etat * pMotionDisable,motion_cmd_set_actuator_kinematics_arg * pmotion_wanted_kinematics,motion_goto_parameter * pgotoparam)
{
	m_pMotionDisable 			= pMotionDisable;
	m_pMotionActuorKinematics 		= pMotionActuorKinematics;
	m_pMotionSpeed 				= pMotionSpeed;
	m_pMotionTrajectory			= pMotionTrajectory;
	m_pmotion_wanted_kinematics 		= pmotion_wanted_kinematics;
	m_pgotoparam				= pgotoparam;
	m_motion_status 			= MOTION_UPDATING_TRAJECTORY;
	m_motion_State 				= MOTION_STATE_ENABLED;
}

////////////////////////////////////////
//méthode virtuelle Effectue l'action de l'etat
//Param :
//retourne: Réussite de l'action		
bool StateMotionEnable::run()
{
	//log_format(LOG_INFO, "Run dans l'etat %s", this->getNameEtat());
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		m_motion_kinematics[i].v = 0;
		m_motion_kinematics[i].mode = KINEMATICS_SPEED;
	}

	motion_update_motors();
	return true;
}		




void StateMotionEnable::motion_set_actuator_kinematics( motion_cmd_set_actuator_kinematics_arg cmd)
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

void StateMotionEnable::motion_goto(VectPlan dest, VectPlan cp, enum motion_way way, enum motion_trajectory_type type, const KinematicsParameters &linearParam, const KinematicsParameters &angularParam)
{
	xSemaphoreTake(m_motion_mutex, portMAX_DELAY);

	log_format(LOG_INFO, "etat %s, motion Goto", this->getNameEtat());
	m_motion_Wanted_State = MOTION_STATE_TRAJECTORY;

	log_format(LOG_INFO, "m_WantedState  %d", m_motion_Wanted_State);
	m_pgotoparam->motion_wanted_dest = loc_to_abs(dest, -cp);
	m_pgotoparam->motion_wanted_trajectory_type = type;
	m_pgotoparam->motion_wanted_way = way;
	m_pgotoparam->motion_wanted_linearParam = linearParam;
	m_pgotoparam->motion_wanted_angularParam = angularParam;
	m_motion_status = MOTION_UPDATING_TRAJECTORY;
	xSemaphoreGive(m_motion_mutex);
}

////////////////////////////////////////
//méthode recupere l'etat suivant
//Param :
//retourne: Id de l'etat suivant		
Etat * StateMotionEnable::getProchainEtat()
{
	Etat * pFuturState = this;
	bool all_op_enable = true;

	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		all_op_enable &= can_motor[i].is_op_enable();
	}

	if( power_get() || ! all_op_enable || m_motion_Wanted_State == MOTION_STATE_DISABLED )
	{
		pFuturState = m_pMotionDisable;
	}

	switch(m_motion_Wanted_State)
	{
		case MOTION_STATE_ACTUATOR_KINEMATICS:
			pFuturState = m_pMotionActuorKinematics;
			break;
		case MOTION_STATE_TRAJECTORY:
			pFuturState = m_pMotionTrajectory;
			break;
		case MOTION_STATE_SPEED:
			pFuturState = m_pMotionSpeed;
			break;
		case MOTION_NONE_STATE: //Pas de Break;
		case MOTION_MAX_STATE: //Pas de Break;
		case MOTION_STATE_DISABLED: //Pas de Break;
		case MOTION_STATE_TRY_ENABLE: //Pas de Break;
		case MOTION_STATE_ENABLED: //Pas de Break;
		case MOTION_STATE_INTERRUPTING: //Pas de Break;
		default:
			break;
	}

	return pFuturState;
}
