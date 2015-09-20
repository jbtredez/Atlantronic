#include "kernel/log.h"
#include "MotionVar.h"
#include "MotionEtat.h"

#ifndef STATE_MOTION_ACTUATOR_KINEMATIC_H
#define STATE_MOTION_ACTUATOR_KINEMATIC_H

class StateMotionActuatorKinematic : public MotionEtat
{
	private:
		Etat * m_pMotionDisable;
		Etat * m_pMotionEnable;

	    motion_cmd_set_actuator_kinematics_arg * m_pmotion_wanted_kinematics; // cinematique desiree (mode MOTION_ACTUATOR_KINEMATICS)

	public:
		StateMotionActuatorKinematic();
		~StateMotionActuatorKinematic();


		void motion_set_actuator_kinematics(motion_cmd_set_actuator_kinematics_arg cmd);
		void InitState(Etat * pMotionEnable, Etat * pMotionDisable, motion_cmd_set_actuator_kinematics_arg * motion_wanted_kinematics){m_pMotionEnable = pMotionEnable;m_pMotionDisable = pMotionDisable; m_pmotion_wanted_kinematics = motion_wanted_kinematics;};

		////////////////////////////////////////
		//méthode virtuelle Effectue l'action de l'etat
		//Param :
		//retourne: Réussite de l'action
		bool run();



		////////////////////////////////////////
		//méthode recupere l'etat suivant
		//Param :
		//retourne: l'etat suivant
		//          null si on ne change pas etat
		Etat * getProchainEtat();
};

#endif /* STATE_MOTION_ACTUATOR_KINEMATIC_H */
