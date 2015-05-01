
#ifndef CLAPET_H
#define CLAPET_H

#include "kernel/stratege_machine/action.h"
class clapet : public action
{
	private :
	public:
	////////////////////////////////////////////////
	/// function    : clapet()
	/// descrition  : constructor
	/// param       : firstcheckpoint : VectPlan first checkpoint of the action
	/// retrun      : none
	////////////////////////////////////////////////
	clapet(VectPlan firstcheckpoint);
	////////////////////////////////////////////////
	/// function    : ~action()
	/// descrition  : destructor
	/// param       : none
	/// retrun      : none
	////////////////////////////////////////////////
	~clapet(){};

	////////////////////////////////////////////////
	/// function    : do_action()
	/// descrition  : execute the action
	/// param       : astratcolor : int the color (GREEN OR YELLOW)
	/// retrun      : -1 if fail or 0 if sucess
	////////////////////////////////////////////////
	int do_action();
}; 

#endif
