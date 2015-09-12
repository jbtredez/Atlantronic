



#ifndef CSTATEMOTIONINTERRRUPTING_H_
#define CSTATEMOTIONINTERRRUPTING_H_
#include "kernel/log.h"
#include "kernel/motion/new_state/MotionVar.h"
#include "kernel/motion/new_state/CMotionEtat.h"
class CStateMotionInterrupting : public MotionEtat
{
	private:
		Etat * m_pMotionEnable;
		Etat * m_pMotionDisable;
	public:
		CStateMotionInterrupting();
		~CStateMotionInterrupting();

		void InitState(Etat * pMotionEnable,Etat * pMotionDisable){m_pMotionEnable = pMotionEnable;m_pMotionDisable = pMotionDisable;};

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
