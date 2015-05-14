

#include "kernel/log.h"
#include "kernel/motion/trajectory.h"
#include "disco/robot_state.h"
#include "elevator.h"
#include "kernel/driver/dynamixel.h"
#include "disco/finger.h"
#include "kernel/location/location.h"
#include "disco/action/gobelet.h"

#include "kernel/stratege_machine/action.h"
#include "kernel/math/vect_plan.h"

#define GOBELET_APPROX_DIST       100

Gobelet::Gobelet(VectPlan firstcheckpoint, const char * name, robotstate * elevator):Action(firstcheckpoint, name)
{
	if(elevator != 0)
	{
		m_elevator =  elevator;
	}
	
	m_actiontype = ACTION_GOBELET;
	 
}

////////////////////////////////////////////////
/// function    : do_action()
/// descrition  : execute the action
/// param       : none
/// retrun      : -1 if fail or 0 if sucess
////////////////////////////////////////////////
int Gobelet::do_action()
{
	int result = -1;
	int essai = 0;

	Action::do_action();

	//Si la reserve n'est pas vide on quitte la fonction
	if(m_elevator->getelevatorstate() != ELEVATOR_EMPTY)
	{
		return -1;
	}

	//On prend un nouvel element
	elevator_set_position(0);
	finger_set_pos(FINGER_OPEN, FINGER_OPEN);

	do 
	{
		trajectory_goto_near_xy(m_firstcheckpoint.x, m_firstcheckpoint.y, GOBELET_APPROX_DIST, WAY_FORWARD, AVOIDANCE_STOP) ;

		if ( trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) == 0)
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

	finger_set_pos(FINGER_GOBLET, FINGER_GOBLET);
	vTaskDelay(300);
	elevator_set_position(10);

	//Vérifie si on a attrapé quelque chose
	//if(ax12.isFlagActive(AX12_LOW_FINGER, DYNAMIXEL_FLAG_STUCK) || ax12.isFlagActive(AX12_HIGH_FINGER, DYNAMIXEL_FLAG_STUCK))
	//{
		m_elevator->setelevatorstate(ELEVATOR_GOBELET);
		m_try = -1;
		return 0;
	//}
	//m_try++;
	//return -1;
}
	
	

