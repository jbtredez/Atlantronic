/*
 * drop_module.h
 *
 *  Created on: May 15, 2017
 *      Author: herzaeone
 */

#ifndef SRC_DISCO_GATE_ACTION_DROP_MODULE_H_
#define SRC_DISCO_GATE_ACTION_DROP_MODULE_H_

#include "middleware/stratege_machine/action.h"
#include "disco/robot_state.h"
#include "middleware/trajectory/Trajectory.h"

class DropModule:
		public Action
{
	private:
		KinematicsParameters m_linParamOrig;
		KinematicsParameters m_angParamOrig;
		uint32_t m_checkpoint;

	public:
		DropModule(VectPlan firstcheckpoint, uint32_t checkpoint, const char * name, RobotState * robot);
		void Initialise(int stratColor);
		int do_action();
		void Exit();
		void slowSpeed(void);
		void resetSpeed(void);
};

#endif /* SRC_DISCO_GATE_ACTION_DROP_MODULE_H_ */
