/*
 * LoadBall.h
 *
 *  Created on: 21 mai 2017
 *      Author: jul
 */

#ifndef LOADBALL_H_
#define LOADBALL_H_

#include "middleware/stratege_machine/action.h"
#include "disco/robot_state.h"
#include "middleware/trajectory/Trajectory.h"

class LoadBall: public Action
{
	public:
		LoadBall(VectPlan firstcheckpoint, uint32_t checkpoint, const char * name, RobotState * robot);
		virtual ~LoadBall();
		int do_action();
		void Exit();
		bool Ready();
};

#endif /* LOADBALL_H_ */
