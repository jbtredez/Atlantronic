#ifndef CLAPET_H
#define CLAPET_H

#include "middleware/stratege_machine/action.h"
#include "disco/star/robot_state.h"

class Clapet : public Action
{
	private :
		RobotState * m_robot;

	public:
		////////////////////////////////////////////////
		/// function    : clapet()
		/// descrition  : constructor
		/// param       : firstcheckpoint : VectPlan first checkpoint of the action
		/// retrun      : none
		////////////////////////////////////////////////
		Clapet(VectPlan firstcheckpoint, const char * name, RobotState * robot);

		////////////////////////////////////////////////
		/// function    : do_action()
		/// descrition  : execute the action
		/// param       : astratcolor : int the color (GREEN OR YELLOW)
		/// retrun      : -1 if fail or 0 if sucess
		////////////////////////////////////////////////
		int do_action();


		////////////////////////////////////////////////
		/// function    : Exit()
		/// descrition  : action effectue pour sortir de l'action proprement
		/// param       : none
		/// retrun      : none
		////////////////////////////////////////////////
		void Exit();
}; 

#endif
