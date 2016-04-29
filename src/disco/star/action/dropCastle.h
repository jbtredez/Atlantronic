#ifndef SRC_DISCO_STAR_ACTION_DROPCASTLE_H_
#define SRC_DISCO_STAR_ACTION_DROPCASTLE_H_


#include "middleware/stratege_machine/action.h"
#include "disco/star/robot_state.h"


class DropCastle: public Action
{
	private :
		RobotState * m_robot;

	public:
		DropCastle(VectPlan firstcheckpoint, const char * name, RobotState * robot);
		void Initialise(int stratcolor);
		int do_action();
		void Exit();


	private:
		int stratColor;

};

#endif /* SRC_DISCO_STAR_ACTION_DROPCASTLE_H_ */
