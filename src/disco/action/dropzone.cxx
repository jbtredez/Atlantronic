

#include "kernel/log.h"
#include "kernel/motion/trajectory.h"
#include "disco/robot_state.h"
#include "elevator.h"

#include "disco/finger.h"
#include "kernel/location/location.h"
#include "disco/action/dropzone.h"

#include "kernel/stratege_machine/action.h"
#include "kernel/math/vect_plan.h"


dropzone::dropzone(VectPlan firstcheckpoint,robotstate * elevator):action(firstcheckpoint)
{
	if(elevator != 0)
	{
		m_elevator = elevator;	
	}
	m_dropposition = 200; 
	m_actiontype = ACTION_DROPZONE;
}

////////////////////////////////////////////////
/// function    : do_action()
/// descrition  : execute the action
/// param       : none
/// retrun      : -1 if fail or 0 if sucess
////////////////////////////////////////////////
int dropzone::do_action()
{
	int result = -1;
	int essai = 0;

	if(m_elevator == 0)
	{
		return -1;
	}


	Eelevator_state state = m_elevator->getelevatorstate();
	VectPlan dropzone = m_firstcheckpoint;
	//Si la reserve est vide ou contient seulement une lumière on quitte la fonction
	if( ( state == ELEVATOR_EMPTY)  && (state == ELEVATOR_LIGHT) )
	{
		return -1;
	}
	
	//On va dans la zone d'entrée de départ
	do 
	{
		trajectory_goto_near(m_firstcheckpoint, DROPSTART_APPROX_DIST, WAY_FORWARD, AVOIDANCE_STOP) ;
		
		if (   trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) == 0)
		{
			result = 0;
		}

		essai++;

		if(essai == 3)
		{
			return -1;
		}
	} while(  result ==-1 ) ;

	essai = 0;
	result = -1;
	do 
	{
		
		if(dropzone.x <0 )
		{
			dropzone.x = dropzone.x - m_dropposition;
			dropzone.theta = 3.14;
			
		}
		else
		{
			dropzone.x = dropzone.x + m_dropposition;
			dropzone.theta = 00.0f;
			
		}

		//on se déplace dans la zone de départ
		trajectory_goto_near(dropzone, DROPSTART_APPROX_DIST, WAY_FORWARD, AVOIDANCE_STOP) ;

		if (   trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) == 0)
		{
			result = 0;
		}

		essai++;

		if(essai == 3)
		{
			return -1;
		}
		
	} while(  result ==-1 ) ;

	essai = 0;

	// faut pas monter l'ascenseur au dela de 100 avec la pince du haut ouverte (sinon, ça frotte sur l'étage du milieu).
	elevator_set_position(100);

	vTaskDelay(100);
	finger_set_pos(FINGER_OPEN, FINGER_OPEN);
	vTaskDelay(400);
	result = -1;
	do 
	{
		trajectory_goto_near(m_firstcheckpoint, DROPSTART_APPROX_DIST, WAY_BACKWARD, AVOIDANCE_STOP) ;
		if (   trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) == 0)
		{
			result = 0;
		}
		essai++;
		if(essai == 3)
		{
			elevator_set_position(0);
			return -1;
		}
	}while(  result ==-1 ) ;



	vTaskDelay(100);

	finger_set_pos(FINGER_CLOSE, FINGER_CLOSE);	
	m_elevator->setnumberelement(0);
 	m_elevator->setelevatorstate(ELEVATOR_EMPTY);
	m_dropposition = m_dropposition - 110;

	return result;
}
	
	

