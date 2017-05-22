/*
 * EscapeStart.h
 *
 *  Created on: Apr 26, 2017
 *      Author: herzaeone
 */

#ifndef SRC_DISCO_GATE_ACTION_ESCAPESTART_H_
#define SRC_DISCO_GATE_ACTION_ESCAPESTART_H_

#include "middleware/stratege_machine/action.h"
#include "disco/robot_state.h"
#include "middleware/trajectory/Trajectory.h"

class EscapeStart:
		public Action
{

	public:
		KinematicsParameters m_linParamOrig;
		KinematicsParameters m_angParamOrig;

	public:
		EscapeStart(VectPlan firstcheckpoint, const char * name, RobotState * robot);
		void Initialise(int stratColor);
		int do_action();
		void Exit();
		void slowSpeed(void);
		void resetSpeed(void);
};

#endif /* SRC_DISCO_GATE_ACTION_ESCAPESTART_H_ */
