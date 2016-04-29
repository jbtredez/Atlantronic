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
	while( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) != 0 )
	{
		bresult = -1;
	}

	// Ouvrir les pinces

	return bresult;
}
