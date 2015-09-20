#ifndef STATE_MOTION_SPEED_H
#define STATE_MOTION_SPEED_H

#include "kernel/log.h"
#include "MotionVar.h"
#include "MotionEtat.h"

class StateMotionSpeed : public MotionEtat
{
	private:
		Etat * m_pMotionDisable;
		Etat * m_pMotionEnable;
	public:
		StateMotionSpeed();
		~StateMotionSpeed();


		void InitState(Etat * pMotionEnable, Etat * pMotionDisable){m_pMotionEnable = pMotionEnable;m_pMotionDisable = pMotionDisable;};

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
		//méthode recupere l'etat suivant
		//Param :
		//retourne: l'etat suivant
		//          null si on ne change pas etat
		Etat * getProchainEtat();
};

#endif /* STATE_MOTION_SPEED_H */
