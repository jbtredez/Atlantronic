
#ifndef DEPOSE_CARPETTE_H
#define DEPOSE_CARPETTE_H

#include "kernel/stratege_machine/action.h"
class deposecarpette : public action
{
	private :
	robotstate * m_robot;
	public:
	////////////////////////////////////////////////
	/// function    : deposecarpette()
	/// descrition  : constructor
	/// param       : firstcheckpoint : VectPlan first checkpoint of the action
	/// retrun      : none
	////////////////////////////////////////////////
	deposecarpette(VectPlan firstcheckpoint,robotstate * robot);

	////////////////////////////////////////////////
	/// function    : ~deposecarpette()
	/// descrition  : destructor
	/// param       : none
	/// retrun      : none
	////////////////////////////////////////////////
	~deposecarpette(){};

	////////////////////////////////////////////////
	/// function    : do_action()
	/// descrition  : execute the action
	/// param       : none
	/// retrun      : -1 if fail or 0 if sucess
	////////////////////////////////////////////////
	int do_action();

}; 

#endif
