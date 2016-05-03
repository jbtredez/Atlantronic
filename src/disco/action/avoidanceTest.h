#ifndef SRC_DISCO_ACTION_AVOIDANCETEST_H_
#define SRC_DISCO_ACTION_AVOIDANCETEST_H_

#include "middleware/stratege_machine/action.h"

class AvoidanceTest : public Action
{
	public:
		////////////////////////////////////////////////
		/// function    : Idle()
		/// descrition  : constructor
		/// param       : firstcheckpoint : VectPlan first checkpoint of the action
		/// retrun      : none
		////////////////////////////////////////////////
		AvoidanceTest(VectPlan firstcheckpoint, const char  * name);

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
		bool sens;
		int stratColor;
};

#endif /* SRC_DISCO_ACTION_AVOIDANCETEST_H_ */
