#ifndef LIGHT_H
#define LIGHT_H

#include "middleware/stratege_machine/action.h"
#include "disco/star/robot_state.h"

#define LIGHT_APPROX_DIST       100

class Light : public Action
{
	private :
		RobotState * m_elevator;
	public:

	////////////////////////////////////////////////
	/// function    : light()
	/// descrition  : constructor
	/// param       : firstcheckpoint : VectPlan first checkpoint of the action
	/// retrun      : none
	////////////////////////////////////////////////
	Light(VectPlan firstcheckpoint, const char * name, RobotState * elevator, bool light2);

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
