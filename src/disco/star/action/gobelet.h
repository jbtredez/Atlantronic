#ifndef GOBELET_H
#define GOBELET_H

#include "middleware/stratege_machine/action.h"

class Gobelet : public Action
{
	private :
		RobotState * m_elevator;

	public:
	////////////////////////////////////////////////
	/// function    : gobelet()
	/// descrition  : constructor
	/// param       : firstcheckpoint : VectPlan first checkpoint of the action
	/// retrun      : none
	////////////////////////////////////////////////
	Gobelet(VectPlan firstcheckpoint, const char * name, RobotState * elevator);

	////////////////////////////////////////////////
	/// function    : do_action()
	/// descrition  : execute the action
	/// param       : none
	/// retrun      : -1 if fail or 0 if sucess
	////////////////////////////////////////////////
	int do_action();
}; 

#endif
