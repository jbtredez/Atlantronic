/*
 * module_harvest.cxx
 *
 *  Created on: May 21, 2017
 *      Author: herzaeone
 */

#include "harvest2.h"
#include "kernel/log.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/match.h"

#include "disco/gate/gate.h"
#include "disco/gate/servos.h"

Harvest2::Harvest2(VectPlan firstcheckpoint, uint32_t checkpoint, const char * name, RobotState * robot):
	Action(firstcheckpoint, name,(void*) robot)
{
	m_checkpoint = checkpoint;
}


void Harvest2::Initialise(int stratColor)
{
	Action::Initialise(stratColor);
	m_stratColor = stratColor;
}

int Harvest2::do_action()
{
	uint32_t actionResult = 0;
	Action::do_action();


	return actionResult;
}

void Harvest2::Exit()
{

}


void Harvest2::slowSpeed(void)
{
	trajectory.getKinematicsParam(&m_linParamOrig, &m_angParamOrig);

	KinematicsParameters linParam = {300, 600, 600};
	KinematicsParameters angParam = m_angParamOrig;
	angParam.vMax /= 2;
	angParam.aMax /= 2;
	angParam.dMax /= 2;

	trajectory.setKinematicsParam(linParam, angParam);

	vTaskDelay(100);
}

void Harvest2::resetSpeed(void)
{
	trajectory.setKinematicsParam(m_linParamOrig, m_angParamOrig);
	vTaskDelay(100);
}
