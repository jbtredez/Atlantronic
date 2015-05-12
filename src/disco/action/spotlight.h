
#ifndef SPOTLIGHT_H
#define SPOTLIGHT_H

#include "kernel/stratege_machine/action.h"

#include "kernel/stratege_machine/action_composite.h"




class spotlight : public actioncomposite
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
	spotlight(VectPlan firstcheckpoint, char * name, robotstate * elevator);
	////////////////////////////////////////////////
	/// function    : ~action()
	/// descrition  : destructor
	/// param       : none
	/// retrun      : none
	////////////////////////////////////////////////
	~spotlight(){};

	////////////////////////////////////////////////
	/// function    : do_action()
	/// descrition  : execute the action
	/// param       : none
	/// retrun      : -1 if fail or 0 if sucess
	////////////////////////////////////////////////
	int do_action();
}; 

#endif
