#ifndef SRC_DISCO_STAR_ACTION_DROPFISHES_H_
#define SRC_DISCO_STAR_ACTION_DROPFISHES_H_

#include "middleware/stratege_machine/action.h"
#include "disco/star/robot_state.h"

class DropFishes:
		public Action
{
	private :
		RobotState * m_robot;

	public:
		DropFishes(VectPlan firstcheckpoint, const char * name, RobotState * robot);
		void Initialise(int stratcolor);
		int do_action();
		void Exit();

	private:
		int m_stratColor;
};

#endif /* SRC_DISCO_STAR_ACTION_DROPFISHES_H_ */
