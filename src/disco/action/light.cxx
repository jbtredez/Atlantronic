

#include "kernel/log.h"
#include "kernel/motion/trajectory.h"
#include "disco/robot_state.h"
#include "elevator.h"
#include "kernel/driver/dynamixel.h"
#include "disco/finger.h"
#include "kernel/location/location.h"
#include "disco/action/light.h"

#include "kernel/stratege_machine/action.h"
#include "kernel/math/vect_plan.h"

light::light(VectPlan firstcheckpoint, const char * name, robotstate * elevator, bool light2):Action(firstcheckpoint, name)
{
	if(elevator != 0)
	{
		m_elevator =  elevator;
	}

	m_light2 = light2;
	m_actiontype = ACTION_LIGHT;
}

////////////////////////////////////////////////
/// function    : do_action()
/// descrition  : execute the action
/// param       : none
/// retrun      : -1 if fail or 0 if sucess
////////////////////////////////////////////////
int light::do_action()
{
	int result = -1;
	int essai = 0;
	int nbelement = m_elevator->getnumberelement();


	Action::do_action();
	
	//Si la reserve est pleine on quitte la fonction
	if(m_elevator->getnumberelement() >= MAX_ELEMENT)
	{
		return -1;
	}

	elevator_set_position(10);
	finger_set_pos(FINGER_HALF_OPEN, FINGER_HALF_OPEN);

	do 
	{
		if( m_light2 )
		{
			trajectory_goto_near(m_firstcheckpoint, 0, WAY_FORWARD, AVOIDANCE_STOP) ;
		}
		else
		{
			trajectory_goto_near(m_firstcheckpoint, 0, WAY_ANY, AVOIDANCE_STOP) ;
		}
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

	if( m_light2 )
	{
		finger_set_pos(FINGER_OPEN, FINGER_OPEN);

		VectPlan dest = m_firstcheckpoint;
		dest.y -= 200;
		result = -1;
		do
		{
			trajectory_goto_near(dest, 0, WAY_ANY, AVOIDANCE_STOP) ;
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
	}

	finger_set_pos(FINGER_CLOSE, FINGER_HALF_CLOSE);
	vTaskDelay(500);





	log(LOG_INFO, "light on board");
	m_elevator->setnumberelement(nbelement + 1);
	m_elevator->setelevatorstate(ELEVATOR_LIGHT);
	m_try = -1;
	finger_set_pos(FINGER_HALF_CLOSE, FINGER_HALF_CLOSE);

	if( m_light2 )
	{
		int res = 0;
		do
		{
			trajectory_goto_near(m_firstcheckpoint, 0, WAY_ANY, AVOIDANCE_STOP);
			res = trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000);
			if( res != 0 )
			{
				vTaskDelay(500);
			}
		}
		while( res != 0);
	}

	return 0;
}
	
	

