#ifndef MOVE_H
#define MOVE_H

#include "middleware/stratege_machine/action.h"
#include "disco/robot_state.h"
#include "middleware/trajectory/Trajectory.h"

class AvoidanceTest : public Action
{
	public:
		AvoidanceTest(VectPlan firstcheckpoint,const char  * name, RobotState * robot);
		////////////////////////////////////////////////
		/// function    : Idle()
		/// descrition  : constructor
		/// param       : firstcheckpoint : VectPlan first checkpoint of the action
		/// retrun      : none
		////////////////////////////////////////////////
		//AvoidanceTest(VectPlan firstcheckpoint, const char  * name, int nretry);

		////////////////////////////////////////////////
		/// function    : do_action()
		/// descrition  : execute the action
		/// param       : none
		/// retrun      : -1 if fail or 0 if sucess
		////////////////////////////////////////////////
		int do_action();

		void Initialise(int stratcolor);

	private:
		VectPlan pts[4];
		bool m_sens;
		int stratColor;
};

#endif /* SRC_DISCO_ACTION_AVOIDANCETEST_H_ */

