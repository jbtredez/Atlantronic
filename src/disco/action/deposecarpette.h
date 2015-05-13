#ifndef DEPOSE_CARPETTE_H
#define DEPOSE_CARPETTE_H

#include "kernel/stratege_machine/action.h"

class DeposeCarpette : public Action
{
	private :
		robotstate * m_robot;
		bool m_right;

	public:
	////////////////////////////////////////////////
	/// function    : deposecarpette()
	/// descrition  : constructor
	/// param       : firstcheckpoint : VectPlan first checkpoint of the action
	/// retrun      : none
	////////////////////////////////////////////////
	DeposeCarpette(VectPlan firstcheckpoint, const char * name, robotstate * robot, bool right);

	////////////////////////////////////////////////
	/// function    : ~deposecarpette()
	/// descrition  : destructor
	/// param       : none
	/// retrun      : none
	////////////////////////////////////////////////
	~DeposeCarpette(){};

	////////////////////////////////////////////////
	/// function    : do_action()
	/// descrition  : execute the action
	/// param       : none
	/// retrun      : -1 if fail or 0 if sucess
	////////////////////////////////////////////////
	int do_action();
	virtual void Initialise(int stratcolor);
}; 

#endif
