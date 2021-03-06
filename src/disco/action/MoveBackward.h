#ifndef MOVE_BACKWARD_H
#define MOVE_BACKWARD_H

#include "middleware/stratege_machine/action.h"
//Action pour reculer à partir d'un point vers le checkpoint


class MoveBackward : public Action
{
	public:

	////////////////////////////////////////////////
	/// function    : movebackward()
	/// descrition  : constructor
	/// param       : firstcheckpoint : VectPlan first checkpoint of the action
	/// retrun      : none
	////////////////////////////////////////////////
	MoveBackward(VectPlan firstcheckpoint, const char * name );

	////////////////////////////////////////////////
	/// function    : do_action()
	/// descrition  : execute the action
	/// param       : none
	/// retrun      : -1 if fail or 0 if sucess
	////////////////////////////////////////////////
	int do_action();
}; 

#endif
