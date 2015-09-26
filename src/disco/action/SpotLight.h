#ifndef SPOTLIGHT_H
#define SPOTLIGHT_H

#include "middleware/stratege_machine/action.h"
#include "middleware/stratege_machine/action_composite.h"
#include "disco/robot_state.h"

class SpotLight : public ActionComposite
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
		SpotLight(VectPlan firstcheckpoint, char * name, RobotState * elevator);

		////////////////////////////////////////////////
		/// function    : do_action()
		/// descrition  : execute the action
		/// param       : none
		/// retrun      : -1 if fail or 0 if sucess
		////////////////////////////////////////////////
		int do_action();
}; 

#endif
