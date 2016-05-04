#include "duneSucker.h"
#include "kernel/log.h"
#include "kernel/match.h"
#include "disco/star/star.h"
//#include "disco/gate/servos.h"


DuneSucker::DuneSucker(VectPlan firstcheckpoint, const char * name, RobotState * robot):Action(firstcheckpoint, name)
{
	if(robot != 0)
	{
		m_robot =  robot;
	}
}


void DuneSucker::Initialise(int stratcolor)
{
	Action::Initialise(stratcolor);
	m_stratColor = stratcolor;

}


int DuneSucker::do_action()
{
	int bresult = 0;
	Action::do_action();
	VectPlan startPoint(-1000, 500, M_PI_2);
	startPoint = startPoint.symetric(m_stratColor);

	vTaskDelay(300);
	trajectory.goTo(startPoint, WAY_FORWARD, AVOIDANCE_GRAPH);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 15000) != 0 )
	{
		bresult = -1;
	}

	return 0;

	// Ouvrir les pinces
	//Servos::setPumpArmState(PUMP_ARM_OPEN);

	this->slowSpeed();

	vTaskDelay(300);
	trajectory.straight(100);
	if( trajectory.wait(TRAJECTORY_STATE_COLISION, 3000) != 0 )
	{
		bresult = -1;
	}
	vTaskDelay(100);

	vTaskDelay(300);
	trajectory.straight(-100);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 3000) != 0 )
	{
		bresult = -1;
	}

	vTaskDelay(300);
	startPoint.theta = M_PI;
	startPoint = startPoint.symetric(m_stratColor);
	trajectory.rotateTo(startPoint.theta);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 4000) != 0 )
	{
		bresult = -1;
	}

	this->resetSpeed();

	return bresult;
}

void DuneSucker::slowSpeed(void)
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

void DuneSucker::resetSpeed(void)
{
	trajectory.setKinematicsParam(m_linParamOrig, m_angParamOrig);
	vTaskDelay(100);
}
