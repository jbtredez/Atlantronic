#ifndef SRC_DISCO_STAR_ACTION_DROPCASTLE_H_
#define SRC_DISCO_STAR_ACTION_DROPCASTLE_H_


#include "middleware/stratege_machine/action.h"
#include "disco/star/robot_state.h"
#include "middleware/trajectory/Trajectory.h"


class DropCastle: public Action
{
	private :
		RobotState * m_robot;
		KinematicsParameters linParamOrig;
		KinematicsParameters angParamOrig;

	public:
		DropCastle(VectPlan firstcheckpoint, const char * name, RobotState * robot);
		void Initialise(int stratcolor);
		int do_action();
		void Exit();
		void slowSpeed(void);
		void resetSpeed(void);


	private:
		int stratColor;

};

#endif /* SRC_DISCO_STAR_ACTION_DROPCASTLE_H_ */
