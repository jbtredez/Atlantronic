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

ModuleHarvest::ModuleHarvest(VectPlan firstcheckpoint, uint32_t checkpoint, const char * name, RobotState * robot):
	Action(firstcheckpoint, name,(void*) robot)
{
	m_checkpoint = checkpoint;

}


void ModuleHarvest::Initialise(int stratColor)
{
	Action::Initialise(stratColor);
	m_stratColor = stratColor;
}

int ModuleHarvest::do_action()
{
	uint32_t actionResult = 0;
	Action::do_action();
	m_firstcheckpoint.x = 500;
	m_firstcheckpoint.y = 600;
	m_firstcheckpoint.theta = -M_PI_2;
	m_firstcheckpoint = m_firstcheckpoint.symetric(m_stratColor);

	log_format(LOG_INFO, "YOLOOOOO debut action");

	VectPlan pos = location.getPosition();
	log_format(LOG_INFO, "goto ====================================> pos = %d, %d, %d", (int) pos.x, (int) pos.y, (int) (pos.theta * 180/M_PI));

	vTaskDelay(1000);
	trajectory.goTo(m_firstcheckpoint, WAY_FORWARD, AVOIDANCE_STOP);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) != 0 )
	{

	}


	//slowSpeed();

	pos = location.getPosition();
	log_format(LOG_INFO, "straight 100 ====================================> pos = %d, %d, %d", (int) pos.x, (int) pos.y, (int) (pos.theta * 180/M_PI));
	trajectory.straight(100);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) != 0 )
	{

	}
	//resetSpeed();

	float cylinderPos = cylinder.getCurrentPosition();
	cylinder.setPosition(cylinderPos - M_PI_2);

	pos = location.getPosition();
	log_format(LOG_INFO, " End goto====================================> pos = %d, %d, %d", (int) pos.x, (int) pos.y, (int) (pos.theta * 180/M_PI));

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
