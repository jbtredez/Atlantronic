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
		int get_try(){return m_try;};
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

		//nb of try of the action -1 if the action it's done
		int m_try;
		VectPlan m_firstcheckpoint;

		int m_actiontype;
		const char * m_name;
		bool initialized;
};

#endif
