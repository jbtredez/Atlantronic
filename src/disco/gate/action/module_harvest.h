/*
 * module_harvest.h
 *
 *  Created on: May 21, 2017
 *      Author: herzaeone
 */

#ifndef SRC_DISCO_GATE_ACTION_MODULE_HARVEST_H_
#define SRC_DISCO_GATE_ACTION_MODULE_HARVEST_H_

#include "middleware/stratege_machine/action.h"
#include "disco/robot_state.h"
#include "middleware/trajectory/Trajectory.h"

class ModuleHarvest: public Action
{
	public:
		KinematicsParameters m_linParamOrig;
		KinematicsParameters m_angParamOrig;
		uint32_t m_checkpoint;

	public:
		ModuleHarvest(VectPlan firstcheckpoint, uint32_t checkpoint, const char * name, RobotState * robot);
		void Initialise(int stratColor);
		int do_action();
		void Exit();
		void slowSpeed(void);
		void resetSpeed(void);
};

#endif /* SRC_DISCO_GATE_ACTION_MODULE_HARVEST_H_ */
