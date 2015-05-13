#ifndef ACTION_COMPOSITE_H
#define ACTION_COMPOSITE_H

//! @file action_composite.h
//! @brief mother class of the action_composite
//! @author Atlantronic

#include "kernel/log.h"
#include "kernel/math/vect_plan.h"
#define NB_MAX_COMPO_ACTION 10
class actioncomposite : public Action
{
	public:
	//nb of try of the action -1 if the action it's done
	int m_try;
	int m_size_actionlist;
	Action * m_list_action[NB_MAX_COMPO_ACTION];
	//first checkpoint 
	VectPlan m_firstcheckpoint;		
	

	int get_try(){return m_try;};
	VectPlan getfirstcheckpoint(){return m_firstcheckpoint;};

	////////////////////////////////////////////////
	/// function    : actioncomposite()
	/// descrition  : constructor
	/// param       : firstcheckpoint : VectPlan first checkpoint of the action
	/// retrun      : none
	////////////////////////////////////////////////	
	actioncomposite(VectPlan firstcheckpoint, char * name);
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
	int add_action(Action * p_action);


	////////////////////////////////////////////////
	/// function    : Initialise()
	/// descrition  : intialise the checkpoint color
	/// param       : none
	/// retrun      : none
	////////////////////////////////////////////////
	virtual void Initialise(int stratcolor){ m_firstcheckpoint.x = stratcolor *  m_firstcheckpoint.x ;};

	////////////////////////////////////////////////
	/// function    : find_action_not_done()
	/// descrition  : find the nearest a type action wich are not done
	/// param       :int : type of the action
	/// retrun      : l'action ou un pt 0
	////////////////////////////////////////////////
	Action * find_action_not_done(int type, VectPlan position);


};
#endif
