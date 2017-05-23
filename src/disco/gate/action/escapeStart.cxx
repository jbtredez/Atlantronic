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
#include "disco/gate/gate.h"
#include "disco/gate/servos.h"

EscapeStart::EscapeStart(VectPlan firstcheckpoint, const char * name, RobotState * robot):
	Action(firstcheckpoint, name,(void*) robot)
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
	float angle;

	//slowSpeed();
	//vTaskDelay(2000);	// On attends que Star sorte de la zone de départ pour éviter les collisions


	VectPlan pos = location.getPosition();
	//log_format(LOG_INFO, "rotate to mpi/2+1====================================> pos = %d, %d, %d", (int) pos.x, (int) pos.y, (int) (pos.theta * 180/M_PI));
	do
	{
		if (m_stratColor == COLOR_BLUE)
		{
			angle = - M_PI_2 + 1;
		} else
		{
			angle = - M_PI_2 - 1;
		}
		trajectory.rotateTo(angle);
	} while( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) != 0);


	pos = location.getPosition();
	//log_format(LOG_INFO, "straight 200 ====================================> pos = %d, %d, %d", (int) pos.x, (int) pos.y, (int) (pos.theta * 180/M_PI));
	do
	{
		// TODO mettre un goTo
		trajectory.straight( 200.0f );
	} while( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) != 0);

	//resetSpeed();

	pos = location.getPosition();
	//log_format(LOG_INFO, "end straight 200====================================> pos = %d, %d, %d", (int) pos.x, (int) pos.y, (int) (pos.theta * 180/M_PI));


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
