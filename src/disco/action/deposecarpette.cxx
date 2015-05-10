

#include "kernel/log.h"
#include "kernel/motion/trajectory.h"
#include "disco/carpet.h"
#include "disco/robot_state.h"
#include "disco/wing.h"

#include "disco/robot_state.h"
#include "kernel/stratege_machine/action.h"
#include "kernel/math/vect_plan.h"
#include "deposecarpette.h"

deposecarpette::deposecarpette(VectPlan firstcheckpoint,robotstate * robot):action(firstcheckpoint)
{
	if(robot != 0)
	{
		m_robot =  robot;
	}
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

	int bresult = 0;
	int essaie =0;

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

	log_format(LOG_INFO , "Action Depose Carpette");
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


	nextToDropCarpette.y = nextToDropCarpette.y - 100;

	//On essaie de se déplacer 3 fois afin d'abandonner
	do
	{
    		trajectory_goto(nextToDropCarpette, WAY_BACKWARD, AVOIDANCE_STOP);
		if(trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 40000) == 0)
		{
			bresult = 0;

		}
		else
		{

			bresult = -1;

		}
	
		essaie++;
	}while(essaie <= 3 && !bresult); 

	//On baisse pour déposer la carpette
	carpet_set_pos(CARPET_DOWN, CARPET_DOWN);
	vTaskDelay(100);
	carpet_set_pos(CARPET_UP, CARPET_DOWN);
	vTaskDelay(100);
	carpet_set_pos(CARPET_UP, CARPET_UP);

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
	
	

