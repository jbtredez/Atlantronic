
#ifndef GOBELET_H
#define GOBELET_H

#include "kernel/stratege_machine/action.h"



class gobelet : public action
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
	gobelet(VectPlan firstcheckpoint, char * name, robotstate * elevator);
	////////////////////////////////////////////////
	/// function    : ~action()
	/// descrition  : destructor
	/// param       : none
	/// retrun      : none
	////////////////////////////////////////////////
	~gobelet(){};

	////////////////////////////////////////////////
	/// function    : do_action()
	/// descrition  : execute the action
	/// param       : none
	/// retrun      : -1 if fail or 0 if sucess
	////////////////////////////////////////////////
	int do_action();
}; 

#endif
