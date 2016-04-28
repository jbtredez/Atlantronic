#include "kernel/log.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/match.h"
#include "disco/star/star.h"
#include "fishes.h"


void fishes_set_pos(enum fishes_type right, enum fishes_type left)
{
	switch(right)
	{
		case CARPET_UP:
			leftFishWing.setGoalPosition(0);
			break;
		case CARPET_DOWN:
			leftFishWing.setGoalPosition(-1);
			break;
		case CARPET_NO_MOVE:
		default:
			break;
	}

	switch(left)
	{
		case CARPET_UP:
			leftFishWing.setGoalPosition(0);
			break;
		case CARPET_DOWN:
			leftFishWing.setGoalPosition(1);
			break;
		case CARPET_NO_MOVE:
		default:
			break;
	}
}

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
	leftFishWing.setGoalLimits(0, 1.4);

	rightFishWing.setTorqueLimit(1);
	rightFishWing.setGoalLimits(0, -1.4);

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

	if(m_try < 0 )
	{
		return 0;
	}

	// On va a la position de l'action
	do
	{
		vTaskDelay(100);
		trajectory.goToNear(m_firstcheckpoint, 100, WAY_BACKWARD, AVOIDANCE_STOP) ;

	}while( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) != 0) ;

	fishes_set_pos(FISHES_DOWN,  FISHES_NO_MOVE);
}

