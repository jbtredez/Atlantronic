#ifndef SRC_DISCO_STAR_ACTION_FELLOWCASTLE_H_
#define SRC_DISCO_STAR_ACTION_FELLOWCASTLE_H_

#include "middleware/stratege_machine/action.h"
#include "disco/star/robot_state.h"


class FellowCastle: public Action
{
	private :
		RobotState * m_robot;

	public:
		FellowCastle(VectPlan firstcheckpoint, const char * name, RobotState * robot);
		void Initialise(int stratcolor);
		int do_action();
		void Exit();


	private:
		int stratColor;

};

#endif /* SRC_DISCO_STAR_ACTION_FELLOWCASTLE_H_ */
