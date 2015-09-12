/*
 * CStateMotionActuatorKinematic.h
 *
 *  Created on: 24 août 2015
 *      Author: jul
 */
#include "kernel/log.h"
#include "kernel/motion/new_state/MotionVar.h"
#include "kernel/motion/new_state/CMotionEtat.h"


#ifndef CSTATEMOTIONACTUATORKINEMATIC_H_
#define CSTATEMOTIONACTUATORKINEMATIC_H_



class CStateMotionActuatorKinematic : public MotionEtat
{
	private:
		Etat * m_pMotionDisable;
		Etat * m_pMotionEnable;

	    motion_cmd_set_actuator_kinematics_arg * m_pmotion_wanted_kinematics; // cinematique desiree (mode MOTION_ACTUATOR_KINEMATICS)

	public:
		CStateMotionActuatorKinematic();
		~CStateMotionActuatorKinematic();


		void motion_set_actuator_kinematics(motion_cmd_set_actuator_kinematics_arg cmd);
		void InitState(Etat * pMotionEnable, Etat * pMotionDisable, motion_cmd_set_actuator_kinematics_arg * motion_wanted_kinematics){m_pMotionEnable = pMotionEnable;m_pMotionDisable = pMotionDisable; m_pmotion_wanted_kinematics = motion_wanted_kinematics;};

		////////////////////////////////////////
		//méthode virtuelle Effectue l'action de l'etat
		//Param :
		//retourne: Réussite de l'action
		bool run();

		////////////////////////////////////////
		//méthode virtuelle Effectue l'action de l'etat
		//Param :
		//retourne: Réussite de l'action
		bool entry();
		////////////////////////////////////////
		//méthode virtuelle Effectue l'action de l'etat
		//Param :
		//retourne: Réussite de l'action
		bool out();

		////////////////////////////////////////
		//méthode recupere l'etat suivant
		//Param :
		//retourne: l'etat suivant
		//          null si on ne change pas etat
		Etat * getProchainEtat();
};

#endif /* CSTATEMOTIONACTUATORKINEMATIC_H_ */
