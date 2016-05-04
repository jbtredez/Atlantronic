#ifndef SRC_DISCO_STAR_ACTION_DUNESUCKER_H_
#define SRC_DISCO_STAR_ACTION_DUNESUCKER_H_

#include "middleware/stratege_machine/action.h"
#include "disco/robot_state.h"
#include "middleware/trajectory/Trajectory.h"

class DuneSucker: public Action
{
	public:
		DuneSucker(VectPlan firstcheckpoint, const char * name, RobotState * robot);
		void Initialise(int stratcolor);
		int do_action();
		void Exit();
		void slowSpeed(void);
		void resetSpeed(void);


	private:
		int m_stratColor;
		RobotState * m_robot;
		KinematicsParameters m_linParamOrig;
		KinematicsParameters m_angParamOrig;

};

#endif /* SRC_DISCO_STAR_ACTION_DUNESUCKER_H_ */
