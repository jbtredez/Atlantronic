#include "dropFishes.h"
#include "kernel/log.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/match.h"
#include "disco/star/star.h"
#include "disco/star/servos.h"


DropFishes::DropFishes(VectPlan firstcheckpoint, const char * name, RobotState * robot):Action(firstcheckpoint, name)
{
	if(robot != 0)
	{
		m_robot =  robot;
	}

	m_actiontype = ACTION_DROP_FISHES;
	m_stratColor = 0;
}


void DropFishes::Initialise(int stratcolor)
{
	Action::Initialise(stratcolor);
	m_stratColor = stratcolor;

	leftFishWing.setTorqueLimit(1);
	leftFishWing.setGoalLimits(-1.4, 1.4);

	rightFishWing.setTorqueLimit(1);
	rightFishWing.setGoalLimits(-1.4, 1.4);

}

int DropFishes::do_action()
{
	int bresult = 0;
	VectPlan netPos(400, -850, M_PI);
	netPos = netPos.symetric(m_stratColor);

	trajectory.goTo(netPos, WAY_FORWARD,AVOIDANCE_STOP);
//	trajectory.goTo(m_firstcheckpoint, WAY_FORWARD,AVOIDANCE_STOP);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) != 0)
	{
		bresult = -1;
	}

	if (bresult != -1)
	{
		vTaskDelay(300);
		if(m_stratColor == COLOR_GREEN)
		{
			Servos::setWingState(WING_MIDDLE, WING_NO_MOVE);
		}
		else
		{
			Servos::setWingState(WING_NO_MOVE, WING_MIDDLE);
		}

		if(m_stratColor == COLOR_GREEN)
		{
			Servos::setFishRemoverState(FISH_REMOVER_SHAKE, FISH_REMOVER_NO_MOVE);
		}else
		{
			Servos::setFishRemoverState(FISH_REMOVER_NO_MOVE, FISH_REMOVER_SHAKE);
		}
		vTaskDelay(100);
		if(m_stratColor == COLOR_GREEN)
		{
			Servos::setWingState(WING_CLOSE, WING_NO_MOVE);
		}else
		{
			Servos::setWingState(WING_NO_MOVE, WING_CLOSE);
		}
	}

	return bresult;
}
