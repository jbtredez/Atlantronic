#ifndef FEET_H
#define FEET_H

#include "middleware/stratege_machine/action.h"

class feet : public Action
{
	private :
		RobotState * m_elevator;
	public:

	////////////////////////////////////////////////
	/// function    : feet()
	/// descrition  : constructor
	/// param       : firstcheckpoint : VectPlan first checkpoint of the action
	/// retrun      : none
	////////////////////////////////////////////////
	feet(VectPlan firstcheckpoint, const char * name, RobotState * elevator);

	////////////////////////////////////////////////
	/// function    : do_action()
	/// descrition  : execute the action
	/// param       : none
	/// retrun      : -1 if fail or 0 if sucess
	////////////////////////////////////////////////
	int do_action();
}; 

#endif
