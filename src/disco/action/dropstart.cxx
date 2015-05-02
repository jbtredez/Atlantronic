

#include "kernel/log.h"
#include "kernel/motion/trajectory.h"
#include "disco/robot_state.h"
#include "elevator.h"

#include "disco/finger.h"
#include "kernel/location/location.h"
#include "disco/action/dropstart.h"

#include "kernel/stratege_machine/action.h"
#include "kernel/math/vect_plan.h"


dropstart::dropstart(VectPlan firstcheckpoint,robotstate * elevator):action(firstcheckpoint)
{
	m_elevator = elevator;
	m_dropposition = 320; 
}

////////////////////////////////////////////////
/// function    : do_action()
/// descrition  : execute the action
/// param       : none
/// retrun      : -1 if fail or 0 if sucess
////////////////////////////////////////////////
int dropstart::do_action()
{
	int result = -1;
	int essai = 0;
	Eelevator_state state = m_elevator->getelevatorstate();
	VectPlan dropzone = m_firstcheckpoint;
	//Si la reserve est vide ou contient seulement une lumière on quitte la fonction
	if( ( state == ELEVATOR_EMPTY)  && (state == ELEVATOR_LIGHT) )
	{
		return -1;
	}
	
	//On va dans la zone d'entrée de départ
	trajectory_goto_near(m_firstcheckpoint, DROPSTART_APPROX_DIST, WAY_FORWARD, AVOIDANCE_STOP) ;
	do 
	{
		
		if(dropzone.x <0 )
		{
			dropzone.x = dropzone.x - m_dropposition;
			dropzone.theta = -3.14;
			
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
			elevator_set_position(0);
			return -1;
		}
		
	} while(  result ==-1 ) ;

	elevator_set_position(200);
	vTaskDelay(800);
		
	m_elevator->setnumberelement(0);
 	m_elevator->setelevatorstate(ELEVATOR_EMPTY);
	m_dropposition = m_dropposition - 110;

	return result;
}
	
	

