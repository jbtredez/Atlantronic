
#ifndef DROPSTART_H
#define DROPSTART_H

#include "kernel/stratege_machine/action.h"


#define DROPSTART_APPROX_DIST       10


class dropstart : public action
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
	dropstart(VectPlan firstcheckpoint,robotstate * elevator);
	////////////////////////////////////////////////
	/// function    : ~action()
	/// descrition  : destructor
	/// param       : none
	/// retrun      : none
	////////////////////////////////////////////////
	~dropstart(){};

	////////////////////////////////////////////////
	/// function    : do_action()
	/// descrition  : execute the action
	/// param       : none
	/// retrun      : -1 if fail or 0 if sucess
	////////////////////////////////////////////////
	int do_action();
}; 

#endif
