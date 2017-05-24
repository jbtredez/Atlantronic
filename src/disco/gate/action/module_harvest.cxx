/*
 * module_harvest.cxx
 *
 *  Created on: May 21, 2017
 *      Author: herzaeone
 */

#include "module_harvest.h"
#include "kernel/log.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/match.h"

#include "disco/gate/gate.h"
#include "disco/gate/servos.h"

ModuleHarvest::ModuleHarvest(VectPlan firstcheckpoint, uint32_t checkpoint, const char * name, RobotState * robot, bool backAtEnd):
	Action(firstcheckpoint, name,(void*) robot)
{
	m_backAtEnd = backAtEnd;
	m_checkpoint = checkpoint;
}


void ModuleHarvest::Initialise(int stratColor)
{
	Action::Initialise(stratColor);
}

int ModuleHarvest::do_action()
{
	uint32_t actionResult = 0;
	Action::do_action();


	trajectory.goTo(m_firstcheckpoint, WAY_FORWARD, AVOIDANCE_STOP);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) != 0 )
	{

	}


	slowSpeed();

	trajectory.straight(150);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) != 0 )
	{

	}

	// TODO Mettre action barrier
	resetSpeed();

	float cylinderPos = cylinder.getCurrentPosition();
	cylinder.setPosition(cylinderPos - M_PI_2);



	if (m_backAtEnd)
	{
		trajectory.straight(-200);
		if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) != 0 )
		{

		}
		resetSpeed();
	}

	return actionResult;
}

void ModuleHarvest::Exit()
{

}


void ModuleHarvest::slowSpeed(void)
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

void ModuleHarvest::resetSpeed(void)
{
	trajectory.setKinematicsParam(m_linParamOrig, m_angParamOrig);
	vTaskDelay(100);
}
