/*
 * jetPackSpread.cxx
 *
 *  Created on: May 24, 2017
 *      Author: herzaeone
 */

#include "jetPackSpread.h"


#include "drop_module.h"
#include "kernel/log.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/match.h"
#include "disco/gate/gate.h"
#include "disco/gate/servos.h"
#include "disco/gate/gate.h"
#include "kernel/driver/esc.h"


JetPackSpread::JetPackSpread(VectPlan firstcheckpoint, const char * name, RobotState * robot):
	Action(firstcheckpoint, name,(void*) robot)
{


}

void JetPackSpread::Initialise(int stratColor)
{
	Action::Initialise(stratColor);
}

int JetPackSpread::do_action()
{
	uint32_t actionResult = 0;
	Action::do_action();

	trajectory.goTo(m_firstcheckpoint, WAY_BACKWARD, AVOIDANCE_STOP);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) != 0 )
	{

	}

	slowSpeed();
	jetPack.setVal(0.1);
	vTaskDelay(2000);
	jetPackGrid.setGoalPosition(-M_PI_2);

	vTaskDelay(300);

	trajectory.straight(-500);
	if( trajectory.wait(TRAJECTORY_STATE_COLISION, 5000) != 0 )
	{

	}
	else
	{
		jetPack.setVal(0.0);
		vTaskDelay(1000);
	}



	trajectory.straight(500);
	if( trajectory.wait(TRAJECTORY_STATE_COLISION, 5000) != 0 )
	{

	}
	resetSpeed();

}

void JetPackSpread::Exit()
{

}


void JetPackSpread::slowSpeed(void)
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

void JetPackSpread::resetSpeed(void)
{
	trajectory.setKinematicsParam(m_linParamOrig, m_angParamOrig);
	vTaskDelay(100);
}
