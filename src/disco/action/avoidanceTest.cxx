#include "avoidanceTest.h"
#include "kernel/log.h"
#include "middleware/trajectory/Trajectory.h"
#include "disco/robot_state.h"
#include "disco/bot.h"
#include "kernel/location/location.h"



AvoidanceTest::AvoidanceTest(VectPlan firstcheckpoint,const char  * name, RobotState * robot):Action(firstcheckpoint, name, (void*) robot)
{
	set_actiontype(ACTION_MOVE);
	pts[0].x = 500;
	pts[0].y = 700;
	pts[0].theta = 0;
	pts[1].x = 0;
	pts[1].y = 700;
	pts[1].theta = 0;
	pts[2].x = 0;
	pts[2].y =  100;
	pts[2].theta = 0;
	pts[3].x = 500;
	pts[3].y = 0;
	pts[3].theta = 0;

	m_sens = true;
}

void AvoidanceTest::Initialise(int stratcolor)
{
	Action::Initialise(stratcolor);
	this->stratColor = stratcolor;
}


////////////////////////////////////////////////
/// function    : do_action()
/// descrition  : execute the action
/// param       : none
/// retrun      : -1 if fail or 0 if success
////////////////////////////////////////////////
int AvoidanceTest::do_action()
{
	int i = 3;
	while (1)
	{
		VectPlan nextPoint;

		if (m_sens == true)
			i++;
		else
			i--;

		if (i > 3)
			i = 0;
		if (i < 0)
			i = 3;

		nextPoint = pts[i].symetric(stratColor);

		log_format(LOG_INFO, "Avoidance test: Go to point: %d [%d ; %d]",i, (int)nextPoint.x, (int)nextPoint.y);

		vTaskDelay(1000);
		trajectory.goToNear(nextPoint, 0, WAY_ANY, AVOIDANCE_STOP) ;

		if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) != 0)
		{
			m_sens = !m_sens;
		}


	}
	return 0;
}
