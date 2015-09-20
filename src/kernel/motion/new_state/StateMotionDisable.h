#ifndef STATE_MOTION_DISABLE_H
#define STATE_MOTION_DISABLE_H

#include "kernel/log.h"
#include "MotionVar.h"
#include "MotionEtat.h"

class StateMotionDisable : public MotionEtat
{
	private:
		Etat * m_pMotionTryEnable;
	public:
		StateMotionDisable();

		void initState(Etat * pMotionTryEnable){m_pMotionTryEnable = pMotionTryEnable;};

		////////////////////////////////////////
		//méthode virtuelle Effectue l'action de l'etat
		//Param :
		//retourne: Réussite de l'action		
		bool run();

		#ifdef MOTION_AUTO_ENABLE
		////////////////////////////////////////
		//méthode virtuelle Effectue l'action de l'etat
		//Param :
		//retourne: Réussite de l'action		
		bool entry();		
		#endif

		////////////////////////////////////////
		//méthode recupere l'etat suivant
		//Param :
		//retourne: l'etat suivant
		//          null si on ne change pas etat
		Etat * getProchainEtat();
};

#endif
