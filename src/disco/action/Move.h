#ifndef MOVE_H
#define MOVE_H

#include "middleware/stratege_machine/action.h"

class Move : public Action
{
	public:
		////////////////////////////////////////////////
		/// function    : Idle()
		/// descrition  : constructor
		/// param       : firstcheckpoint : VectPlan first checkpoint of the action
		/// retrun      : none
		////////////////////////////////////////////////
		Move(VectPlan firstcheckpoint, const char  * name);
		Move(VectPlan firstcheckpoint, const char  * name, int nretry);

		////////////////////////////////////////////////
		/// function    : do_action()
		/// descrition  : execute the action
		/// param       : none
		/// retrun      : -1 if fail or 0 if sucess
		////////////////////////////////////////////////
		int do_action();
}; 

#endif

