#include "kernel/log.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/match.h"
#include "disco/carpet.h"
#include "disco/wing.h"
#include "clapet.h"

Clapet::Clapet(VectPlan firstcheckpoint, const char * name, RobotState * robot):Action(firstcheckpoint, name)
{
	if(robot != 0)
	{
		m_robot =  robot;
	}
	
	m_actiontype = ACTION_CLAP;
}

////////////////////////////////////////////////
/// function    : do_action()
/// descrition  : execute the action
/// param       : none
/// retrun      : -1 if fail or 0 if sucess
////////////////////////////////////////////////
int Clapet::do_action()
{
	int bresult = 0;
	int essai = 0;
	int result = -1;

	VectPlan nextToClap ;
	wing_cmd_type left = WING_PARK ;
	wing_cmd_type right = WING_PARK ;


	Action::do_action();

	if(m_try < 0 )
	{
		return 0;
	}

	nextToClap = m_firstcheckpoint;

	log_format(LOG_INFO , "Action clapet");
	//On se déplace sur le vecteur y vers l'origine de la taille du clap (un décalage postif si le x est négatif)
	if(m_firstcheckpoint.x > 0)
	{
		nextToClap.theta = -3.14f;

		left = WING_OPEN;

	}
	else
	{
		nextToClap.theta = 0;
		right = WING_OPEN;

	}
	if(m_robot->getcarpetstate() == CARPET_UP)
	{
		carpet_set_pos(CARPET_UP, CARPET_UP);
		vTaskDelay(100);
	}

	do 
	{
		trajectory.goToNearXy(m_firstcheckpoint.x,m_firstcheckpoint.y, 0, WAY_FORWARD, AVOIDANCE_GRAPH);
		if (trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 40000) == 0)
		{
			result = 0;
			log_format(LOG_INFO, "WAIT NEW ORDER");
		}
		else
		{	
			essai++;
		}


		if(essai == 3)
		{
			return -1;
		}

		vTaskDelay(100);	
	} while(  0 != result ) ;



	wing_set_position(left, right);
	m_robot->setwingstate(left,right);


	vTaskDelay(300);

	trajectory.rotateTo(nextToClap.theta);
	if(trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 40000) != 0)
	{
		m_try++;
		return -1;
	}

	//On ferme l'aile pas besoin de réfléchir
	wing_set_position(WING_PARK, WING_PARK);
	m_robot->setwingstate(WING_PARK,WING_PARK);

	vTaskDelay(100);

	return bresult;
}
