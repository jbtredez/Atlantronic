/*
 * jetPackSpread.h
 *
 *  Created on: May 24, 2017
 *      Author: herzaeone
 */

#ifndef SRC_DISCO_GATE_ACTION_JETPACKSPREAD_H_
#define SRC_DISCO_GATE_ACTION_JETPACKSPREAD_H_


#include "middleware/stratege_machine/action.h"
#include "disco/robot_state.h"
#include "middleware/trajectory/Trajectory.h"


class JetPackSpread: public Action
{
	private:
		KinematicsParameters m_linParamOrig;
		KinematicsParameters m_angParamOrig;
		uint32_t m_checkpoint;

	public:
		JetPackSpread(VectPlan firstcheckpoint, const char * name, RobotState * robot);
		void Initialise(int stratColor);
		int do_action();
		void Exit();
		void slowSpeed(void);
		void resetSpeed(void);
};

#endif /* SRC_DISCO_GATE_ACTION_JETPACKSPREAD_H_ */
