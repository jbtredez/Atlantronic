



#ifndef CSTATEMOTIONDISABLE_H_
#define CSTATEMOTIONDISABLE_H_
#include "kernel/log.h"
#include "kernel/motion/new_state/MotionVar.h"
#include "kernel/motion/new_state/CMotionEtat.h"
class CStateMotionDisable : public MotionEtat
{
	private:
		Etat * m_pMotionTryEnable;
	public:
		CStateMotionDisable();
		~CStateMotionDisable();

		void InitState(Etat * pMotionTryEnable){m_pMotionTryEnable = pMotionTryEnable;};

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




#endif
