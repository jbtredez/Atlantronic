/*
 * jetpackSuck.cxx
 *
 *  Created on: May 24, 2017
 *      Author: herzaeone
 */

#include "jetPackSuck.h"
#include "drop_module.h"
#include "kernel/log.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/match.h"
#include "disco/gate/gate.h"
#include "disco/gate/servos.h"
#include "disco/gate/gate.h"
#include "kernel/driver/esc.h"


JetPackSuck::JetPackSuck(VectPlan firstcheckpoint, const char * name, RobotState * robot):
	Action(firstcheckpoint, name,(void*) robot)
{


}

void JetPackSuck::Initialise(int stratColor)
{
	Action::Initialise(stratColor);
}

int JetPackSuck::do_action()
{
	uint32_t actionResult = 0;
	Action::do_action();

	trajectory.goTo(m_firstcheckpoint, WAY_BACKWARD, AVOIDANCE_STOP);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) != 0 )
	{

	}

	// Open grid
	jetPackGrid.setGoalPosition(-M_PI_2);

	vTaskDelay(1000);


	slowSpeed();
	trajectory.straight(-150);

	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) != 0 )
	{

	}


	// Start vent
	jetPack.setVal(0.6);
	vTaskDelay(3000);
	// Slow vent
	jetPack.setVal(0.1);
	jetPackGrid.setGoalPosition(-M_PI_2);
	vTaskDelay(200);
	trajectory.straight(150);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) != 0 )
	{

	}
	// close grid
	vTaskDelay(500);
	jetPackGrid.setGoalPosition(M_PI_2);
	vTaskDelay(500);
	// Stop vent
//	jetPack.setVal(0.0);

	resetSpeed();
}

void JetPackSuck::Exit()
{

}


void JetPackSuck::slowSpeed(void)
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

void JetPackSuck::resetSpeed(void)
{
	trajectory.setKinematicsParam(m_linParamOrig, m_angParamOrig);
	vTaskDelay(100);
}
