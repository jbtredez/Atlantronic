#include "kernel/log.h"
#include "kernel/motion/trajectory.h"
#include "kernel/match.h"
#include "disco/carpet.h"
#include "disco/robot_state.h"
#include "disco/wing.h"

#include "disco/robot_state.h"
#include "kernel/stratege_machine/action.h"
#include "kernel/math/vect_plan.h"
#include "deposecarpette.h"

deposecarpette::deposecarpette(VectPlan firstcheckpoint, const char * name,robotstate * robot, bool right):action(firstcheckpoint,name)
{
	if( m_robot != 0)
	{
		m_robot =  robot;
	}
	m_right = right;
	m_actiontype = ACTION_CARPET;
}

////////////////////////////////////////////////
/// function    : do_action()
/// descrition  : execute the action
/// param       : none
/// retrun      : -1 if fail or 0 if sucess
////////////////////////////////////////////////
int deposecarpette::do_action()
{
	action::do_action();
	int bresult = 0;

	VectPlan nextToDropCarpette ;

	if(m_try < 0 )
	{
		return 0;
	}

	if(m_robot->getwingopen() == WING_OPEN)
	{
		m_robot->setwingstate(WING_PARK,WING_PARK);
		vTaskDelay(100);
	}
	nextToDropCarpette = m_firstcheckpoint;

	//On se déplace prés de l'escalier et on se met en position
	nextToDropCarpette.theta = -1.57f;



	//Mise en place de la position
	trajectory_goto(nextToDropCarpette, WAY_FORWARD, AVOIDANCE_STOP);

	//Si on arrive pas à joindre la position on abandonne
	if(trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 40000) != 0)
	{
		m_try++;
		return -1; 
	}


	nextToDropCarpette.y = nextToDropCarpette.y + 80;

	//On essaie de se déplacer 3 fois avant d'abandonner
	trajectory_goto(nextToDropCarpette, WAY_BACKWARD, AVOIDANCE_STOP);
	trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 40000);
	// pas de verif target reached, on est peut etre en collision avec la marche.

	//On baisse pour déposer la carpette
	if( m_right)
	{
		carpet_set_pos(CARPET_DOWN, CARPET_NO_MOVE);
	}
	else
	{
		carpet_set_pos(CARPET_NO_MOVE, CARPET_DOWN);
	}

	vTaskDelay(1000);
	if( m_right)
	{
		carpet_set_pos(CARPET_UP, CARPET_NO_MOVE);
	}
	else
	{
		carpet_set_pos(CARPET_NO_MOVE, CARPET_UP);
	}

	int result = -1;
	int essai = 0;
	do
	{
		trajectory_goto_near_xy(m_firstcheckpoint.x, m_firstcheckpoint.y, 0, WAY_ANY, AVOIDANCE_STOP);

		if ( trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) == 0)
		{
			result = 0;
		}
		essai++;
		if(essai == 3)
		{
			m_try++;
			return -1;
		}
	} while(  result ==-1 ) ;

	if(bresult == 0)
	{
		m_try = -1;
	}
	else
	{
		m_try++;				
	}

	return bresult;
}


