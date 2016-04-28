#ifndef FISHES_H_
#define FISHES_H_

#include "middleware/stratege_machine/action.h"
#include "disco/star/robot_state.h"

enum fishes_type
{
	FISHES_UP,
	FISHES_DOWN,
	FISHES_NO_MOVE,
};

class Fishes  : public Action
{
	private :
		RobotState * m_robot;

	public:
		////////////////////////////////////////////////
		/// function    : Fishes()
		/// descrition  : constructor
		/// param       : firstcheckpoint : VectPlan first checkpoint of the action
		/// retrun      : none
		////////////////////////////////////////////////
		Fishes(VectPlan firstcheckpoint, const char * name, RobotState * robot);

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

#endif /* FISHES_H_ */
