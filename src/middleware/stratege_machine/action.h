#ifndef ACTION_H
#define ACTION_H

//! @file action.h
//! @brief mother class of the action
//! @author Atlantronic

#include "kernel/log.h"
#include "kernel/math/VectPlan.h"

#define ACTION_NONE		-1

class Action
{
	public:
		int get_try(){return m_retry;};
		VectPlan get_firstcheckpoint(){return m_firstcheckpoint;};
		const char* get_name(){return m_name;};

		int get_actiontype(){return m_actiontype;};
		void set_actiontype(int actiontype){m_actiontype = actiontype;};

		////////////////////////////////////////////////
		/// function    : action()
		/// descrition  : constructor
		/// param       : firstcheckpoint : VectPlan first checkpoint of the action
		/// retrun      : none
		////////////////////////////////////////////////
		Action(VectPlan firstcheckpoint, const char * name);

		////////////////////////////////////////////////
		/// function    : Exit()
		/// descrition  : action effectue pour sortir de l'action proprement
		/// param       : none
		/// retrun      : none
		////////////////////////////////////////////////
		void Exit(){ };


		////////////////////////////////////////////////
		/// function    : Initialise()
		/// descrition  : intialise the checkpoint color
		/// param       : none
		/// retrun      : none
		////////////////////////////////////////////////
		virtual void Initialise(int stratcolor);
		////////////////////////////////////////////////
		/// function    : do_action()
		/// descrition  : execute the action
		/// param       : none
		/// retrun      : -1 if fail or 0 if sucess
		////////////////////////////////////////////////
		virtual int do_action();

		// Nombre de re-tentative en cas d'échec: 0: on retente pas, 1 on retente une fois,...
		// Strat décrémente à chaque essai, arrivé à -1 elle n'est plus appelée
		int m_retry;
		VectPlan m_firstcheckpoint;

		int m_actiontype;
		const char * m_name;
		bool initialized;
};

#endif
