/*
 * CStateMotionTryEnable.h
 *
 *  Created on: 23 août 2015
 *      Author: jul
 */
#include "kernel/log.h"
#include "kernel/motion/new_state/MotionVar.h"
#include "CMotionEtat.h"

#ifndef CSTATEMOTIONTRYENABLE_H_
#define CSTATEMOTIONTRYENABLE_H_


class CStateMotionTryEnable: public MotionEtat
{
	private:
		Etat * m_pMotionDisable;
		Etat * m_pMotionEnable;
	public:
		CStateMotionTryEnable();
		~CStateMotionTryEnable();

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

#endif /* CSTATEMOTIONTRYENABLE_H_ */
