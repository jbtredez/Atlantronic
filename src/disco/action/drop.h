
#ifndef DROPSTART_H
#define DROPSTART_H

#include "kernel/stratege_machine/action.h"


#define DROPSTART_APPROX_DIST       100


class drop : public action
{
	private :
	robotstate * m_elevator;
	int m_dropposition;
	public:

	////////////////////////////////////////////////
	/// function    : dropstart()
	/// descrition  : constructor
	/// param       : firstcheckpoint : VectPlan first checkpoint of the action
	/// retrun      : none
	////////////////////////////////////////////////
	drop(VectPlan firstcheckpoint,robotstate * elevator);
	////////////////////////////////////////////////
	/// function    : ~action()
	/// descrition  : destructor
	/// param       : none
	/// retrun      : none
	////////////////////////////////////////////////
	~drop(){};

	////////////////////////////////////////////////
	/// function    : do_action()
	/// descrition  : execute the action
	/// param       : none
	/// retrun      : -1 if fail or 0 if sucess
	////////////////////////////////////////////////
	int do_action();
}; 

#endif
