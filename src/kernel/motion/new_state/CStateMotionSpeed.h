/*
 * CStateMotionSpeed.h
 *
 *  Created on: 24 août 2015
 *      Author: jul
 */

#ifndef CSTATEMOTIONSPEED_H_
#define CSTATEMOTIONSPEED_H_
#include "kernel/log.h"
#include "kernel/motion/new_state/MotionVar.h"
#include "kernel/motion/new_state/CMotionEtat.h"

class CStateMotionSpeed : public MotionEtat
{
	private:
		Etat * m_pMotionDisable;
		Etat * m_pMotionEnable;
	public:
		CStateMotionSpeed();
		~CStateMotionSpeed();


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

#endif /* CMOTIONSPEED_H_ */
