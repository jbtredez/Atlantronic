#ifndef STATE_MOTION_INTERRRUPTING_H
#define STATE_MOTION_INTERRRUPTING_H

#include "kernel/log.h"
#include "MotionVar.h"
#include "MotionEtat.h"

class StateMotionInterrupting : public MotionEtat
{
	private:
		Etat * m_pMotionEnable;
		Etat * m_pMotionDisable;
	public:
		StateMotionInterrupting();
		~StateMotionInterrupting();

		void InitState(Etat * pMotionEnable,Etat * pMotionDisable){m_pMotionEnable = pMotionEnable;m_pMotionDisable = pMotionDisable;};

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

#endif
