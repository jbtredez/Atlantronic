#include "dropCastle.h"
#include "kernel/log.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/match.h"
#include "disco/star/star.h"
#include "disco/star/servos.h"

DropCastle::DropCastle(VectPlan firstcheckpoint, const char * name, RobotState * robot):Action(firstcheckpoint, name)
{
	if(robot != 0)
	{
		m_robot =  robot;
	}
	m_actiontype = ACTION_DROP_CASTLE;
}


void DropCastle::Initialise(int stratcolor)
{
	Action::Initialise(stratcolor);
	this->stratColor = stratcolor;

}


int DropCastle::do_action()
{
	int bresult = 0;
	Action::do_action();

	vTaskDelay(300);
	trajectory.goTo(m_firstcheckpoint, WAY_FORWARD, AVOIDANCE_GRAPH);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) != 0 )
	{
		bresult = -1;
	}

	// Ouvrir les pinces
	Servos::setDoorsState(DOOR_OPEN);

	this->slowSpeed();

	vTaskDelay(300);
	trajectory.straight(-200);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) != 0 )
	{
		bresult = -1;
	}
	vTaskDelay(100);

	this->resetSpeed();

	return bresult;
}

void DropCastle::slowSpeed(void)
{
	trajectory.getKinematicsParam(&this->linParamOrig, &this->angParamOrig);

	KinematicsParameters linParam = {300, 600, 600};
	KinematicsParameters angParam = angParamOrig;
	angParam.vMax /= 2;
	angParam.aMax /= 2;
	angParam.dMax /= 2;

	trajectory.setKinematicsParam(linParam, angParam);

	vTaskDelay(100);
}

void DropCastle::resetSpeed(void)
{
	trajectory.setKinematicsParam(this->linParamOrig, this->angParamOrig);
	vTaskDelay(100);
}
