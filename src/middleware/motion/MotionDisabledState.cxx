#include "MotionDisabledState.h"
#include "kernel/driver/power.h"

MotionDisabledState::MotionDisabledState() :
	StateMachineState("MOTION_DISABLED",MOTION_DISABLED)
{

}

void MotionDisabledState::entry(void* data)
{
	Motion* m = (Motion*) data;
	// Par defaut quand on entre dans un nouvel etat on ne veut pas aller plus loin
	m->m_wantedState = MOTION_UNKNOWN_STATE;
#ifdef MOTION_AUTO_ENABLE
	// si on veut aller dans l'état MOTION_ENABLED automatiquement
	m->m_wantedState  = MOTION_ENABLED;
#endif
}

void MotionDisabledState::run(void* data)
{
	Motion* m = (Motion*) data;
	for(int i = 0; i < CAN_MOTOR_MAX; i++)
	{
		m->m_kinematics[i].pos = m->m_kinematicsMes[i].pos;
		m->m_kinematics[i].v = 0;
		m->m_canMotor[i].enable(false);
	}
}

unsigned int MotionDisabledState::transition(void* data)
{
	Motion* m = (Motion*) data;

	// changement d'état vers enable possible si on a de la puissance
	if( m->m_wantedState == MOTION_ENABLED && !power_get() )
	{
		return MOTION_TRY_ENABLE;
	}

	return m_stateId;
}
