/*
 * module_harvest.cxx
 *
 *  Created on: May 21, 2017
 *      Author: herzaeone
 */

#include "nul.h"
#include "kernel/log.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/match.h"

#include "disco/gate/gate.h"
#include "disco/gate/servos.h"

Nul::Nul(VectPlan firstcheckpoint, uint32_t checkpoint, const char * name, RobotState * robot):
	Action(firstcheckpoint, name,(void*) robot)
{
	m_checkpoint = checkpoint;
}


void Nul::Initialise(int stratColor)
{
	Action::Initialise(stratColor);
	m_stratColor = stratColor;
}

int Nul::do_action()
{
	VectPlan nulPoints[4] = {VectPlan(200,500,-M_PI_2), VectPlan(200,100,0), VectPlan(500,0,0), VectPlan(500,600,M_PI_2)};

	uint32_t actionResult = 0;
	Action::do_action();

//	VectPlan pos = location.getPosition();

	for(int i = 0; i < 4; i++ )
	{
		nulPoints[i] = nulPoints[i].symetric(m_stratColor);
		trajectory.goTo(nulPoints[i], WAY_FORWARD, AVOIDANCE_STOP);
		if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) != 0 )
		{

		}
	}

	trajectory.straight(-200);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) != 0 )
	{

	}


	return actionResult;
}

void Nul::Exit()
{

}


void Nul::slowSpeed(void)
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

void Nul::resetSpeed(void)
{
	trajectory.setKinematicsParam(m_linParamOrig, m_angParamOrig);
	vTaskDelay(100);
}
