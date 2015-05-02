#ifndef ACTION_COMPOSITE_H
#define ACTION_COMPOSITE_H

//! @file action_composite.h
//! @brief mother class of the action_composite
//! @author Atlantronic

#include "kernel/log.h"
#include "kernel/math/vect_plan.h"
#define NB_MAX_COMPO_ACTION 10
class actioncomposite : public action
{
	public:
	//nb of try of the action -1 if the action it's done
	int m_try;
	action * List_action[NB_MAX_COMPO_ACTION];
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
	actioncomposite(VectPlan firstcheckpoint);
	////////////////////////////////////////////////
	/// function    : ~action()
	/// descrition  : destructor
	/// param       : none
	/// retrun      : none
	////////////////////////////////////////////////
	~actioncomposite(){};

	////////////////////////////////////////////////
	/// function    : add_action()
	/// descrition  : add action in the list of action
	/// param       : pointeur sur l'objet action
	/// retrun      : result of the operation -1 fail, 0 sucess
	////////////////////////////////////////////////
	int add_action(action * p_action)


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
	virtual int  do_action();
	
};

#endif
