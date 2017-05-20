#ifndef SRC_DISCO_STAR_ACTION_DUNECASTLE_H_
#define SRC_DISCO_STAR_ACTION_DUNECASTLE_H_

#include "middleware/stratege_machine/action.h"
#include "disco/robot_state.h"
#include "middleware/trajectory/Trajectory.h"


class DuneCastle: public Action
{
	private :
		RobotState * m_robot;
		KinematicsParameters linParamOrig;
		KinematicsParameters angParamOrig;
		int stratColor;

	public:
		DuneCastle(VectPlan firstcheckpoint, const char * name, RobotState * robot);
		void Initialise(int stratcolor);
		int do_action();
		void Exit();
		void slowSpeed(void);
		void resetSpeed(void);
};

#endif /* SRC_DISCO_STAR_ACTION_DUNECASTLE_H_ */
