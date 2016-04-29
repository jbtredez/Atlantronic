#include "kernel/log.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/match.h"
#include "disco/star/star.h"
#include "disco/star/servos.h"
#include "fishes.h"




////////////////////////////////////////////////
/// function    : Fishes()
/// description  : constructor
/// param       : firstcheckpoint: The action starting point
/// param       : name: The name of the action
/// param       : robot: The instance of the robot which executes the action
/// retrun      : none
////////////////////////////////////////////////
Fishes::Fishes(VectPlan firstcheckpoint, const char * name, RobotState * robot):Action(firstcheckpoint, name)
{
	if(robot != 0)
	{
		m_robot =  robot;
	}

	m_actiontype = ACTION_FISHES;

	log_format(LOG_INFO, "Fishes: %d : %d : %f", firstcheckpoint.x, firstcheckpoint.y, firstcheckpoint.theta);
}

////////////////////////////////////////////////
/// function    : Initialise()
/// description  : Initialises the action
/// param       : none
/// retrun      : none
////////////////////////////////////////////////
void Fishes::Initialise(int stratcolor)
{
	Action::Initialise(stratcolor);
	this->stratColor = stratcolor;

	leftFishWing.setTorqueLimit(1);
	leftFishWing.setGoalLimits(-1.4, 1.4);

	rightFishWing.setTorqueLimit(1);
	rightFishWing.setGoalLimits(-1.4, 1.4);

}

////////////////////////////////////////////////
/// function    : do_action()
/// description  : execute the action
/// param       : none
/// retrun      : -1 if fail or 0 if success
////////////////////////////////////////////////
int Fishes::do_action()
{
	int bresult = 0;
	Action::do_action();
	VectPlan dest(900, -850, M_PI);
	dest = dest.symetric(stratColor);

	if(m_try < 0 )
	{
		return 0;
	}

	// On va a la position de l'action
	do
	{
		vTaskDelay(100);
		trajectory.goTo(dest, WAY_FORWARD, AVOIDANCE_STOP) ;

	}while( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) != 0) ;

	if(stratColor == COLOR_GREEN)
		Servos::setWingState(WING_OPEN, WING_CLOSE);
	else
		Servos::setWingState(WING_CLOSE, WING_OPEN);

	vTaskDelay(500);
	trajectory.straight(200);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) != 0)
	{
		bresult = 1;
	}

	vTaskDelay(300);
	if(stratColor == COLOR_GREEN)
		Servos::setWingState(WING_MIDDLE, WING_CLOSE);
	else
		Servos::setWingState(WING_CLOSE, WING_MIDDLE);

	vTaskDelay(500);
	VectPlan netPos(400, -850, M_PI);
	netPos = netPos.symetric(stratColor);
	// A mettre dans une sous strat
	trajectory.goTo(netPos, WAY_FORWARD,AVOIDANCE_STOP);

	vTaskDelay(500);
	if(stratColor == COLOR_GREEN)
		Servos::setWingState(WING_OPEN, WING_CLOSE);
	else
		Servos::setWingState(WING_CLOSE, WING_OPEN);

	return bresult;
}

