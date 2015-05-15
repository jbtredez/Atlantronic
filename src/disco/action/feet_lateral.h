
#ifndef FEET_LATERAL_H
#define FEET_LATERAL_H

#include "kernel/stratege_machine/action.h"



class FeetLateral : public Action
{
	private :
	robotstate * m_elevator;
	public:

	////////////////////////////////////////////////
	/// function    : feet()
	/// descrition  : constructor
	/// param       : firstcheckpoint : VectPlan first checkpoint of the action
	/// retrun      : none
	////////////////////////////////////////////////
	FeetLateral(VectPlan firstcheckpoint, const char * name, robotstate * elevator);
	////////////////////////////////////////////////
	/// function    : ~action()
	/// descrition  : destructor
	/// param       : none
	/// retrun      : none
	////////////////////////////////////////////////
	~FeetLateral(){};

	////////////////////////////////////////////////
	/// function    : do_action()
	/// descrition  : execute the action
	/// param       : none
	/// retrun      : -1 if fail or 0 if sucess
	////////////////////////////////////////////////
	int do_action();

	VectPlan p1;
	VectPlan p2;
}; 

#endif
