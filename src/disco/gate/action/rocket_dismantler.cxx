/*
 * rocket_dismantler.cxx
 *
 *  Created on: May 8, 2017
 *      Author: herzaeone
 */

#include "rocket_dismantler.h"
#include "kernel/log.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/match.h"
#include "disco/star/star.h"
#include "disco/star/servos.h"

RocketDismantler::RocketDismantler(VectPlan firstcheckpoint, uint32_t checkpoint, const char * name, RobotState * robot):
	Action(firstcheckpoint, name)
{
	m_checkpoint = checkpoint;
}

void RocketDismantler::Initialise(int stratColor)
{
	Action::Initialise(stratColor);
	m_stratColor = stratColor;
}

int RocketDismantler::do_action()
{
	uint32_t actionResult = 0;
	Action::do_action();

	slowSpeed();
	vTaskDelay(300);

	// Go to the wall
	do
	{
		if(m_stratColor == COLOR_YELLOW)
		{
			m_checkpoint++;
		}

		// First get to position via graph

		vTaskDelay(100);
		trajectory.goToGraphNode(m_checkpoint,0,WAY_FORWARD, AVOIDANCE_STOP);
		if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) != 0 )
		{
			break;
		}

		vTaskDelay(100);
		m_firstcheckpoint = m_firstcheckpoint.symetric(m_stratColor);
		trajectory.rotateTo(m_firstcheckpoint.theta);
		if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) != 0 )
		{
			break;
		}

		// StraighToWall ne passe pas avec linux donc:

		vTaskDelay(100);
		trajectory.straight(200);
		if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) != 0 )
		{
			break;
		}

		// back of 15 mm
		vTaskDelay(100);
		trajectory.straight(-15);
		if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) != 0 )
		{
			break;
		}


		// Activate actuators
	} while(false);

	// extract from the wall
	vTaskDelay(100);
	trajectory.straight(-150);

	resetSpeed();
	vTaskDelay(100);


	return actionResult;
}

void RocketDismantler::Exit()
{

}


void RocketDismantler::slowSpeed(void)
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

void RocketDismantler::resetSpeed(void)
{
	trajectory.setKinematicsParam(m_linParamOrig, m_angParamOrig);
	vTaskDelay(100);
}
