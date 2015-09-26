#ifndef DROPSTART_H
#define DROPSTART_H

#include "middleware/stratege_machine/action.h"
#include "disco/robot_state.h"

#define DROP_APPROX_DIST       10

class Drop : public Action
{
	private :
		RobotState * m_elevator;
	public:

	////////////////////////////////////////////////
	/// function    : drop()
	/// descrition  : constructor
	/// param       : firstcheckpoint : VectPlan first checkpoint of the action
	/// retrun      : none
	////////////////////////////////////////////////
	Drop(VectPlan firstcheckpoint, char * name, RobotState * elevator);

	////////////////////////////////////////////////
	/// function    : do_action()
	/// descrition  : execute the action
	/// param       : none
	/// retrun      : -1 if fail or 0 if sucess
	////////////////////////////////////////////////
	int do_action();
}; 

#endif
