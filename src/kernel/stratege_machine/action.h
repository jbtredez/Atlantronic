#ifndef ACTION_H
#define ACTION_H

//! @file action.h
//! @brief mother class of the action
//! @author Atlantronic

#include "kernel/log.h"
#include "kernel/math/vect_plan.h"

class action
{
	public:
	//nb of try of the action -1 if the action it's done
	int m_try;
	//first checkpoint 
	VectPlan m_firstcheckpoint;		
	

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
	/// function    : Initialise()
	/// descrition  : intialise the checkpoint color
	/// param       : none
	/// retrun      : none
	////////////////////////////////////////////////
	virtual void Initialise(int stratcolor){ m_firstcheckpoint.x = stratcolor *  m_firstcheckpoint.x ;};
	////////////////////////////////////////////////
	/// function    : do_action()
	/// descrition  : execute the action
	/// param       : none
	/// retrun      : -1 if fail or 0 if sucess
	////////////////////////////////////////////////
	virtual int  do_action(){log_format(LOG_INFO , "Action mere");return -1;};
	
};

#endif
