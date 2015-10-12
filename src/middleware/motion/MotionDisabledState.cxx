#include "MotionDisabledState.h"
#include "kernel/driver/power.h"

MotionDisabledState::MotionDisabledState() :
	StateMachineState("MOTION_DISABLED",MOTION_DISABLED)
{

}

void MotionDisabledState::entry(void* data)
{
//Par defaut quand on entre dans un nouvel etat on ne veut pas aller plus loin
	m->m_wantedState = MOTION_UNKNOWN_STATE;
#ifndef MOTION_AUTO_ENABLE
	//Par defaut on ne fait rien
//	m->m_enableWanted = MOTION_ENABLE_WANTED_UNKNOWN;
#else
	//Si on veut aller dans l'état AUTO_ENABLE cad => on veut aller dans l'etat ENABLE pas besoin de variables suplémentaires
	Motion* m = (Motion*) data;
	m->m_wantedState  = MOTION_ENABLE_STATE;
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

//#ifndef MOTION_AUTO_ENABLE
	//Cas d'erreur
	// Bah on ne fait rien car c'est dans cette Etat qu'on étient le moteur pas la peine de faire quelque chose
	//if( power_get() )
	//{
		// puissance desactivee
	//	m->m_enableWanted = MOTION_ENABLE_WANTED_UNKNOWN;
	//}
//#else
///  Même si on pas de puissance 
///   Cette partie est inutile car elle est géré dans la méthode d'entry
///	if( ! power_get() )
//	{
//		m->m_enableWanted = MOTION_ENABLE_WANTED_ON;
//	}
//#endif
	///Quand on allume les moteur c'est pour aller dans l'etat Enable (c'est évident non????? ) on ne garde pas la motion allumage des moteurs ou on change le nom MOTION_TRY_ENABLE => MOTION_TRY_ENABLE_MOTOR
	///Même si on a de la puissance on veut changer d'état on change d'état que veut tu faire d'autre ??? 
	//Sinon c'est un cas de blocage on restera toujours dans le même état
	if( m->m_wantedState == MOTION_ENABLE &&  !power_get() )
	{
		return MOTION_TRY_ENABLE;
	}

	return m_stateId;
}
