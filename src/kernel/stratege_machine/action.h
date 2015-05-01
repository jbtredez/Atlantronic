#ifndef ACTION_H
#define ACTION_H

//! @file action.h
//! @brief mother class of the action
//! @author Atlantronic

#include "kernel/math/vect_plan.h"

class action
{
	protected:
	//nb of try of the action -1 if the action it's done
	int m_try;
	//first checkpoint 
	VectPlan m_firstcheckpoint;		
	
	public :
	int get_try(){return m_try;};
	VectPlan get_firstcheckpoint(){return m_firstcheckpoint;};

	////////////////////////////////////////////////
	/// function    : action()
	/// descrition  : constructor
	/// param       : firstcheckpoint : VectPlan first checkpoint of the action
	/// retrun      : none
	////////////////////////////////////////////////	
	action(VectPlan firstcheckpoint);
	////////////////////////////////////////////////
	/// function    : ~action()
	/// descrition  : destructor
	/// param       : none
	/// retrun      : none
	////////////////////////////////////////////////
	~action(){};

	////////////////////////////////////////////////
	/// function    : do_action()
	/// descrition  : execute the action
	/// param       : none
	/// retrun      : -1 if fail or 0 if sucess
	////////////////////////////////////////////////
	int  do_action(){return -1;};
	
};

#endif
