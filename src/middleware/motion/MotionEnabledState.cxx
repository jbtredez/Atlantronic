#include "MotionEnabledState.h"
#include "kernel/log.h"
#include "kernel/driver/power.h"
#include "kernel/match.h"

MotionEnabledState::MotionEnabledState() :
	StateMachineState("MOTION_ENABLED",MOTION_ENABLED)
{

}

void MotionEnabledState::entry(void* data)
{
	Motion* m = (Motion*) data;

	//Satisfaction de la volonte operateur ou de la volonte automatique
	m->m_wantedState = MOTION_UNKNOWN_STATE;
}

void MotionEnabledState::run(void* data)
{
	Motion* m = (Motion*) data;
	for(int i = 0; i < MOTION_MOTOR_MAX; i++)
	{
		m->m_kinematics[i].v = 0;
		m->m_kinematics[i].mode = KINEMATICS_SPEED;
	}

	m->motionUpdateMotors();
}

unsigned int MotionEnabledState::transition(void* data)
{
	Motion* m = (Motion*) data;
	bool all_op_enable = true;

	for(int i = 0; i < MOTION_MOTOR_MAX; i++)
	{
		all_op_enable &= m->m_motionMotor[i]->is_op_enable();
	}

	// si Aucun moteur actif
	if(power_get() || !all_op_enable  || match_end != 0)
	{
		return MOTION_DISABLED;
	}

	// action operateur de changer d'etat dans les etat suivant on retourne l'etat MotionEnable
	if( m->m_wantedState == MOTION_DISABLED 
	 || m->m_wantedState == MOTION_ACTUATOR_KINEMATICS
	 || m->m_wantedState == MOTION_SPEED
	 || m->m_wantedState == MOTION_TRAJECTORY)
	{
		//log_format(LOG_INFO, "WantedState %d ",m->m_wantedState);
		return m->m_wantedState; 
	}

	// sinon dans les autres cas on change d'etat
	return m_stateId;
}
