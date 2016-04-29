#include "fellowCastle.h"
#include "kernel/log.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/match.h"
#include "disco/star/star.h"
#include "disco/star/servos.h"

FellowCastle::FellowCastle(VectPlan firstcheckpoint, const char * name, RobotState * robot):Action(firstcheckpoint, name)
{
	if(robot != 0)
	{
		m_robot =  robot;
	}
	m_actiontype = ACTION_FELLOW_CASTLE;
}


void FellowCastle::Initialise(int stratcolor)
{
	Action::Initialise(stratcolor);
	this->stratColor = stratcolor;

}


int FellowCastle::do_action()
{
	int bresult = 0;
	Action::do_action();
	VectPlan checkpoint(1100.0, 100.0, M_PI);
	checkpoint = checkpoint.symetric(stratColor);


	trajectory.goTo(checkpoint, WAY_FORWARD, AVOIDANCE_GRAPH);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) != 0 )
	{
		bresult = -1;
		goto free;
	}

	// Ouvrir les pinces

	//Avancer
	vTaskDelay(300);
	trajectory.straight(300);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 3000) != 0 )
	{
		bresult = -1;
	}

free:
	return bresult;
}
