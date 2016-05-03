#ifndef HUT_H
#define HUT_H

#include "middleware/stratege_machine/action.h"
#include "disco/robot_state.h"

class Hut : public Action
{
	private :
		RobotState * m_robot;

	public:
		////////////////////////////////////////////////
		/// function    : Hut()
		/// descrition  : constructor
		/// param       : firstcheckpoint : VectPlan first checkpoint of the action
		/// retrun      : none
		////////////////////////////////////////////////
		Hut(VectPlan firstcheckpoint, const char * name, RobotState * robot);

		////////////////////////////////////////////////
		/// function    : Initialise()
		/// descrition  : intialise the action color
		/// param       : none
		/// retrun      : none
		////////////////////////////////////////////////
		void Initialise(int stratcolor);

		////////////////////////////////////////////////
		/// function    : do_action()
		/// descrition  : execute the action
		/// param       : astratcolor : int the color (GREEN OR PURPLE)
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


	private:
		int goToWall();

		int stratColor;
}; 

#endif	// HUT_H
