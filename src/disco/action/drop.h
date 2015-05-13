
#ifndef DROPSTART_H
#define DROPSTART_H

#include "kernel/stratege_machine/action.h"

#define DROP_APPROX_DIST       10

class drop : public Action
{
	private :
	robotstate * m_elevator;
	public:

	////////////////////////////////////////////////
	/// function    : drop()
	/// descrition  : constructor
	/// param       : firstcheckpoint : VectPlan first checkpoint of the action
	/// retrun      : none
	////////////////////////////////////////////////
	drop(VectPlan firstcheckpoint, char * name, robotstate * elevator);
	////////////////////////////////////////////////
	/// function    : ~drop()
	/// descrition  : destructor
	/// param       : none
	/// retrun      : none
	////////////////////////////////////////////////
	~drop(){};

	////////////////////////////////////////////////
	/// function    : do_action()
	/// descrition  : execute the action
	/// param       : none
	/// retrun      : -1 if fail or 0 if sucess
	////////////////////////////////////////////////
	int do_action();



}; 

#endif
