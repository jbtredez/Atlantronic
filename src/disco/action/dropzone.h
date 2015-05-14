
#ifndef DROPSTART_H
#define DROPSTART_H

#include "kernel/stratege_machine/action.h"


#define DROPSTART_APPROX_DIST       10


class DropZone : public Action
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
	DropZone(VectPlan firstcheckpoint, const char * name, robotstate * elevator);
	////////////////////////////////////////////////
	/// function    : ~action()
	/// descrition  : destructor
	/// param       : none
	/// retrun      : none
	////////////////////////////////////////////////
	~DropZone(){};

	////////////////////////////////////////////////
	/// function    : Exit()
	/// descrition  : action effectue pour sortir de l'action proprement
	/// param       : none
	/// retrun      : none
	////////////////////////////////////////////////
	void Exit();


	////////////////////////////////////////////////
	/// function    : do_action()
	/// descrition  : execute the action
	/// param       : none
	/// retrun      : -1 if fail or 0 if sucess
	////////////////////////////////////////////////
	int do_action();
}; 

#endif
