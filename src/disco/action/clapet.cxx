

#include "kernel/log.h"
#include "kernel/motion/trajectory.h"
#include "kernel/match.h"
#include "disco/carpet.h"
#include "disco/wing.h"

#include "disco/robot_state.h"

#include "clapet.h"

#include "kernel/stratege_machine/action.h"
#include "kernel/math/vect_plan.h"

Clapet::Clapet(VectPlan firstcheckpoint, const char * name, robotstate * robot):Action(firstcheckpoint, name)
{
	if(robot != 0)
	{
		m_robot =  robot;
	}
	
	m_actiontype = ACTION_CLAP;
}


////////////////////////////////////////////////
/// function    : Exit()
/// descrition  : action effectue pour sortir de l'action proprement
/// param       : none
/// retrun      : none
////////////////////////////////////////////////
void Clapet::Exit()
{ 
	

	do
	{
		vTaskDelay(100);
		trajectory_goto_near_xy(m_firstcheckpoint.x, m_firstcheckpoint.y, 0, WAY_BACKWARD, AVOIDANCE_STOP);
	}while( trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 40000) != 0 );


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
	int essaie =0;
	int second_x_position = 0;

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
		second_x_position = m_firstcheckpoint.x - 185;
		nextToClap.theta = -3.14f;

		left = WING_OPEN;

	}
	else
	{
		second_x_position = m_firstcheckpoint.x + 185;
		nextToClap.theta = 0;
		right = WING_OPEN;

	}
	if(m_robot->getcarpetstate() == CARPET_UP)
	{
		carpet_set_pos(CARPET_UP, CARPET_UP);
		vTaskDelay(100);
	}


	//Mise en place de la position
	trajectory_goto(nextToClap, WAY_FORWARD, AVOIDANCE_STOP);

	//On ouvre nos ailes, pas besoin de réflechir de quel coté on est(homologation).

	//Si on arrive pas à joindre le clapet on abandonne
	if(trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 40000) != 0)
	{
		m_try++;
		return -1; 
	}


	//On n'ouvre pas l'aile besoin de réfléchir

	wing_set_position(left, right);
	m_robot->setwingstate(left,right);	

	nextToClap.x = second_x_position;

	//On essaie de se déplacer 3 fois afin d'abandonner
	do
	{
		trajectory_goto(nextToClap, WAY_FORWARD, AVOIDANCE_STOP);
		if(trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 40000) == 0)
		{
			bresult = 0;
			m_try = -1;
		}
		else
		{

			bresult = -1;
			m_try++;
		}

		essaie++;
	}while(essaie <= 3 && !bresult); 


	//On ferme l'aile pas besoin de réfléchir
	wing_set_position(WING_PARK, WING_PARK);
	m_robot->setwingstate(WING_PARK,WING_PARK);

	vTaskDelay(100);

	Exit();

	return bresult;
}

