#include "kernel/log.h"
#include "kernel/motion/new_state/MotionVar.h"
#include "MotionEtat.h"

#ifndef STATE_MOTION_TRY_ENABLE_H
#define STATE_MOTION_TRY_ENABLE_H

class StateMotionTryEnable: public MotionEtat
{
	private:
		Etat * m_pMotionDisable;
		Etat * m_pMotionEnable;
	public:
		StateMotionTryEnable();
		~StateMotionTryEnable();

		void InitState(Etat * pMotionEnable, Etat * pMotionDisable){m_pMotionDisable = pMotionDisable;m_pMotionEnable = pMotionEnable;};

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

#endif /* STATE_MOTION_TRY_ENABLE_H */
