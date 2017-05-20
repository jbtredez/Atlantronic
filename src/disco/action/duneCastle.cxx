#include "duneCastle.h"
#include "kernel/log.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/match.h"
#include "disco/star/star.h"
#include "disco/star/servos.h"

DuneCastle::DuneCastle(VectPlan firstcheckpoint, const char * name, RobotState * robot):
	Action(firstcheckpoint, name)
{
	if(robot != 0)
	{
		m_robot =  robot;
	}
	m_actiontype = ACTION_FELLOW_CASTLE;
}


void DuneCastle::Initialise(int stratcolor)
{
	Action::Initialise(stratcolor);
	this->stratColor = stratcolor;

}


int DuneCastle::do_action()
{
	int bresult = 0;
	Action::do_action();
	VectPlan checkpoint(1000, 830, M_PI);
	checkpoint = checkpoint.symetric(stratColor);


	trajectory.goTo(checkpoint, WAY_FORWARD, AVOIDANCE_GRAPH);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) != 0 )
	{
		bresult = -1;
		goto free;
	}

	// Ouvrir les pinces
	if(stratColor == COLOR_BLUE)
		Servos::setWingState(WING_NO_MOVE, WING_OPEN);
	else
		Servos::setWingState(WING_OPEN, WING_NO_MOVE);

	this->slowSpeed();

	//Avancer
	vTaskDelay(300);
	trajectory.straight(200);
	if( trajectory.wait(TRAJECTORY_STATE_COLISION, 3000) != 0 )
	{
		bresult = -1;
	}

	// Position de serrage des pinces
	vTaskDelay(300);
	Servos::setDoorsState(DOOR_GRIP);
	vTaskDelay(300);

	// Reculer
	vTaskDelay(300);
	trajectory.straight(-150);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 3000) != 0 )
	{
		bresult = -1;
	}

free:
	this->resetSpeed();
	return bresult;
}


void DuneCastle::slowSpeed(void)
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

void DuneCastle::resetSpeed(void)
{
	trajectory.setKinematicsParam(this->linParamOrig, this->angParamOrig);
	vTaskDelay(100);
}
