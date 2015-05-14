
#ifndef LIGHT_H
#define LIGHT_H

#include "kernel/stratege_machine/action.h"

#define LIGHT_APPROX_DIST       100

class light : public Action
{
	private :
	robotstate * m_elevator;
	public:

	////////////////////////////////////////////////
	/// function    : light()
	/// descrition  : constructor
	/// param       : firstcheckpoint : VectPlan first checkpoint of the action
	/// retrun      : none
	////////////////////////////////////////////////
	light(VectPlan firstcheckpoint, const char * name, robotstate * elevator, bool light2);
	////////////////////////////////////////////////
	/// function    : ~action()
	/// descrition  : destructor
	/// param       : none
	/// retrun      : none
	////////////////////////////////////////////////
	~light(){};

	////////////////////////////////////////////////
	/// function    : do_action()
	/// descrition  : execute the action
	/// param       : none
	/// retrun      : -1 if fail or 0 if sucess
	////////////////////////////////////////////////
	int do_action();
	bool m_light2;
}; 

#endif
