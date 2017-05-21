/*
 * drop_module.cxx
 *
 *  Created on: May 15, 2017
 *      Author: herzaeone
 */

#include "drop_module.h"
#include "kernel/log.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/match.h"
#include "disco/star/star.h"
#include "disco/star/servos.h"

DropModule::DropModule(VectPlan firstcheckpoint, uint32_t checkpoint, const char * name, RobotState * robot):
	Action(firstcheckpoint, name,(void*) robot)
{


}

void DropModule::Initialise(int stratColor)
{
	Action::Initialise(stratColor);
	m_stratColor = stratColor;
}

int DropModule::do_action()
{
	uint32_t actionResult = 0;
	Action::do_action();

	do
	{
		vTaskDelay(100);
		trajectory.goToGraphNode((m_stratColor == COLOR_BLUE)? 0: 1,0,WAY_FORWARD, AVOIDANCE_STOP);
		if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) != 0 )
		{
			break;
		}

		vTaskDelay(100);
		m_firstcheckpoint.theta = M_PI_4/2;
		m_firstcheckpoint = m_firstcheckpoint.symetric(m_stratColor);
		trajectory.rotateTo(m_firstcheckpoint.theta);
		if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) != 0 )
		{
			break;
		}

		slowSpeed();
		vTaskDelay(100);
		trajectory.straight(200);
		if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) != 0 )
		{
			break;
		}

	} while(false);

	vTaskDelay(100);
	do
	{
		trajectory.straight(-150);
	} while (trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) != 0);

	resetSpeed();
	vTaskDelay(100);

}

void DropModule::Exit()
{

}


void DropModule::slowSpeed(void)
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

void DropModule::resetSpeed(void)
{
	trajectory.setKinematicsParam(m_linParamOrig, m_angParamOrig);
	vTaskDelay(100);
}
