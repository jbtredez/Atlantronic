

#include "kernel/log.h"
#include "kernel/motion/trajectory.h"
#include "disco/robot_state.h"
#include "elevator.h"

#include "disco/finger.h"
#include "kernel/location/location.h"
#include "disco/action/feet.h"

#include "kernel/stratege_machine/action.h"
#include "kernel/math/vect_plan.h"

#define FEET_APPROX_DIST       100
feet::feet(VectPlan firstcheckpoint,robotstate * elevator):action(firstcheckpoint)
{
m_elevator = elevator;
	 
}

////////////////////////////////////////////////
/// function    : do_action()
/// descrition  : execute the action
/// param       : none
/// retrun      : -1 if fail or 0 if sucess
////////////////////////////////////////////////
int feet::do_action()
{
	int result = -1;
	int essai = 0;
	int nbelement = m_elevator->getnumberelement();


	//S ila reserve est pleine on quitte la fonction
	if(m_elevator->getnumberelement() >= MAX_ELEMENT)
	{
		return -1;
	}
	//On souleve la pile pour ajouter le nouvel element 
	elevator_set_position(100);

	do 
	{
		trajectory_goto_near(m_firstcheckpoint, FEET_APPROX_DIST, WAY_FORWARD, AVOIDANCE_STOP) ;

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

	finger_set_pos(FINGER_OPEN, FINGER_OPEN);
	vTaskDelay(500);
	elevator_set_position(0);
	vTaskDelay(800);
	finger_set_pos(FINGER_CLOSE, FINGER_CLOSE);
	vTaskDelay(200);
	
	m_elevator->setnumberelement(nbelement + 1);
 	m_elevator->setelevatorstate(ELEVATOR_FEET);

	return result;
}
	
	

