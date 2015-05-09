
#ifndef FEED_H
#define FEED_H

#include "kernel/stratege_machine/action.h"


#include "kernel/stratege_machine/action_composite.h"

class feed : public actioncomposite
{
	private :
	robotstate * m_elevator;
	public:

	////////////////////////////////////////////////
	/// function    : gobelet()
	/// descrition  : constructor
	/// param       : firstcheckpoint : VectPlan first checkpoint of the action
	/// retrun      : none
	////////////////////////////////////////////////
	feed(VectPlan firstcheckpoint,robotstate * elevator);
	////////////////////////////////////////////////
	/// function    : ~action()
	/// descrition  : destructor
	/// param       : none
	/// retrun      : none
	////////////////////////////////////////////////
	~feed(){};

	////////////////////////////////////////////////
	/// function    : do_action()
	/// descrition  : execute the action
	/// param       : none
	/// retrun      : -1 if fail or 0 if sucess
	////////////////////////////////////////////////
	int do_action();
}; 

#endif
