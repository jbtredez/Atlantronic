

#include "kernel/log.h"
#include "kernel/motion/trajectory.h"
#include "disco/robot_state.h"
#include "elevator.h"

#include "disco/finger.h"
#include "kernel/location/location.h"
#include "disco/action/gobelet.h"

#include "kernel/stratege_machine/action.h"
#include "kernel/math/vect_plan.h"

#define GOBELET_APPROX_DIST       100
gobelet::gobelet(VectPlan firstcheckpoint,robotstate * elevator):action(firstcheckpoint)
{
m_elevator = elevator;
	 
}

////////////////////////////////////////////////
/// function    : do_action()
/// descrition  : execute the action
/// param       : none
/// retrun      : -1 if fail or 0 if sucess
////////////////////////////////////////////////
int gobelet::do_action()
{
	int result = -1;
	int essai = 0;


	//S ila reserve est pleine on quitte la fonction
	if(m_elevator->getelevatorstate() != ELEVATOR_EMPTY)
	{
		return -1;
	}
	//On prend un nouvel element
	elevator_set_position(200);
	vTaskDelay(800);
	do 
	{
		trajectory_goto_near(m_firstcheckpoint, GOBELET_APPROX_DIST, WAY_FORWARD, AVOIDANCE_STOP) ;

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
	elevator_set_position(100);
	vTaskDelay(800);
	finger_set_pos(FINGER_CLOSE, FINGER_CLOSE);
	vTaskDelay(200);
	
 	m_elevator->setelevatorstate(ELEVATOR_GOBELET);

	return result;
}
	
	

