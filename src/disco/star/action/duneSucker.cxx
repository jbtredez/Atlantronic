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
	m_state = DUNE_SUCKER_GRAB;
	m_stratColor = 0;
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
	VectPlan startPoint(180, 750, M_PI_2);
	startPoint = startPoint.symetric(m_stratColor);
	m_retry = 3;
	int local_retry = 3;
	bool run = true;


	while(run)
	{
		switch (m_state)
		{
			case DUNE_SUCKER_GOTO_ZONE:
				trajectory.goTo(startPoint, WAY_FORWARD, AVOIDANCE_GRAPH);
				if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 15000) == 0 )
				{
					m_state = DUNE_SUCKER_EXTRACT;
				} else
				{
					bresult = -1;
					run = false;
				}
				break;

			case DUNE_SUCKER_GRAB:
				// Ouvrir les pinces
				//Servos::setPumpArmState(PUMP_ARM_OPEN);
				trajectory.straight(200);
				if( trajectory.wait(TRAJECTORY_STATE_COLISION, 15000) == 0 )
				{
					m_state = DUNE_SUCKER_EXTRACT;
				} else
				{
					// Fermer les pinces
					//Servos::setPumpArmState(PUMP_ARM_UP);
					m_state = DUNE_SUCKER_GOTO_ZONE;
					bresult = -1;
					run = false;
				}
				break;

			case DUNE_SUCKER_EXTRACT:
				trajectory.straight(100);
				if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 15000) == 0 )
				{
					m_state = DUNE_SUCKER_LEAVE;
				} else
				{
					bresult = -1;
					run = false;
				}
				break;

			case DUNE_SUCKER_LEAVE:
				break;

			case DUNE_SUCKER_DROP:
				break;

			default:
				break;
		}
		vTaskDelay(200);
	}


	vTaskDelay(300);
	trajectory.straight(-200);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 3000) != 0 )
	{
		bresult = -1;
	}

	vTaskDelay(300);
	startPoint.x = 800;
	startPoint.y = 400;
	startPoint = startPoint.symetric(m_stratColor);
	trajectory.goToNearXy(startPoint.x, startPoint.y, 100, WAY_BACKWARD, AVOIDANCE_GRAPH);
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
