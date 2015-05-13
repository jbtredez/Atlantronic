
#ifndef CLAPET_H
#define CLAPET_H

#include "kernel/stratege_machine/action.h"
class Clapet : public Action
{
	private :
		robotstate * m_robot;

	public:

		////////////////////////////////////////////////
		/// function    : clapet()
		/// descrition  : constructor
		/// param       : firstcheckpoint : VectPlan first checkpoint of the action
		/// retrun      : none
		////////////////////////////////////////////////
		Clapet(VectPlan firstcheckpoint, const char * name, robotstate * robot);
		////////////////////////////////////////////////
		/// function    : ~action()
		/// descrition  : destructor
		/// param       : none
		/// retrun      : none
		////////////////////////////////////////////////
		~Clapet(){};

		////////////////////////////////////////////////
		/// function    : do_action()
		/// descrition  : execute the action
		/// param       : astratcolor : int the color (GREEN OR YELLOW)
		/// retrun      : -1 if fail or 0 if sucess
		////////////////////////////////////////////////
		int do_action();
}; 

#endif
