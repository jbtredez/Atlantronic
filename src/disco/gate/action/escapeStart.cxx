/*
 * EscapeStart.cpp
 *
 *  Created on: Apr 26, 2017
 *      Author: herzaeone
 */

#include "escapeStart.h"
#include "kernel/log.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/match.h"
#include "disco/star/star.h"
#include "disco/star/servos.h"

EscapeStart::EscapeStart(VectPlan firstcheckpoint, const char * name, RobotState * robot):
	Action(firstcheckpoint, name)
{
	m_actiontype = ACTION_ESCAPE_BASE;

}

void EscapeStart::Initialise(int stratColor)
{
	Action::Initialise(stratColor);
	m_stratColor = stratColor;
}

int EscapeStart::do_action()
{
	uint32_t actionResult = 0;
	Action::do_action();

	slowSpeed();
	vTaskDelay(300);

	trajectory.straight( 800.0f );
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) != 0 )
	{
		actionResult = -1;
	}

	resetSpeed();
	vTaskDelay(100);


	return actionResult;
}

void EscapeStart::Exit()
{

}


void EscapeStart::slowSpeed(void)
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

void EscapeStart::resetSpeed(void)
{
	trajectory.setKinematicsParam(m_linParamOrig, m_angParamOrig);
	vTaskDelay(100);
}
