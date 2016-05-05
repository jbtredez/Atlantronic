#include "fishing.h"
#include "kernel/log.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/match.h"
#include "disco/star/star.h"
#include "disco/star/servos.h"


Fishing::Fishing(VectPlan firstcheckpoint, const char * name, RobotState * robot):Action(firstcheckpoint, name)
{
	if(robot != 0)
	{
		m_robot =  robot;
	}

	m_actiontype = ACTION_FISHING;
	m_stratColor = 0;
}


void Fishing::Initialise(int stratcolor)
{
	Action::Initialise(stratcolor);
	m_stratColor = stratcolor;

	leftFishWing.setTorqueLimit(1);
	leftFishWing.setGoalLimits(-1.4, 1.4);

	rightFishWing.setTorqueLimit(1);
	rightFishWing.setGoalLimits(-1.4, 1.4);

}

int Fishing::do_action()
{
	int bresult = 0;
	Action::do_action();
	VectPlan dest(900, -850, M_PI);
	dest = dest.symetric(m_stratColor);

	if(m_retry < 0 )
	{
		return 0;
	}

	// On va a la position de l'action
	vTaskDelay(100);
	trajectory.goTo(dest, WAY_ANY, AVOIDANCE_STOP) ;

	if ( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) != 0)
	{
		bresult = -1;
	}

	// Si on est arrivé à destination
	if(bresult != -1)
	{
		if(m_stratColor == COLOR_GREEN)
			Servos::setWingState(WING_OPEN, WING_NO_MOVE);
		else
			Servos::setWingState(WING_NO_MOVE, WING_OPEN);

		vTaskDelay(300);
		trajectory.straight(250	);
		if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) != 0)
		{
			bresult = -1;
		}
	}

	// Sortie d'action: on referme les actionneurs et on retourne l'action (que l'on ait réussi ou non)
	vTaskDelay(300);
	if(m_stratColor == COLOR_GREEN)
		Servos::setWingState(WING_CLOSE, WING_NO_MOVE);
	else
		Servos::setWingState(WING_NO_MOVE, WING_CLOSE);

	vTaskDelay(300);
	return bresult;
}
