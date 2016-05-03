#ifndef MOVE_H
#define MOVE_H

#include "middleware/stratege_machine/action.h"

class Move : public Action
{
	public:
		////////////////////////////////////////////////
		/// function    : Idle()
		/// descrition  : constructor
		/// param       : firstcheckpoint : VectPlan first checkpoint of the action
		/// retrun      : none
		////////////////////////////////////////////////
		Move(VectPlan firstcheckpoint, const char  * name);
		Move(VectPlan firstcheckpoint, const char  * name, int nretry);

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
>>>>>>> -Mise à Jour:src/disco/action/avoidanceTest.h
