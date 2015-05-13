

#include "kernel/log.h"
#include "kernel/motion/trajectory.h"
#include "disco/robot_state.h"
#include "elevator.h"
#include "kernel/driver/dynamixel.h"
#include "disco/finger.h"
#include "kernel/location/location.h"
#include "disco/action/feed.h"

#include "kernel/stratege_machine/action.h"
#include "kernel/math/vect_plan.h"

feed::feed(VectPlan firstcheckpoint,char * name, robotstate * elevator):actioncomposite(firstcheckpoint, name)
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
int feed::do_action()
{
	Eelevator_state elevator_state = m_elevator->getelevatorstate();
	VectPlan position = location_get_position();
	Action * p_action;


	Action::do_action();

	switch(elevator_state)
	{
		//Cas où l'acsenceur est vide
		case ELEVATOR_EMPTY:
		{
			p_action = find_action_not_done(ACTION_GOBELET,position);
			if(p_action != 0)
			{
				return p_action->do_action();
			}
			else
			{
				return -1;
			}
			break;
		}

		case ELEVATOR_GOBELET:
		{
			p_action = find_action_not_done(ACTION_FEET,position);
			if(p_action != 0)
			{
				return p_action->do_action();
			}
			else
			{
				p_action = find_action_not_done(ACTION_DROPZONE,position);
				if(p_action != 0)
				{
					return p_action->do_action();
				}
				break;
			}
			break;
		}
		
		///Dans les cas d'échec
		case ELEVATOR_LIGHT:
		case ELEVATOR_FEET:
		default :
			log_format(LOG_INFO , "Action présente non utilisée");

	}

	return -1;	

}
	
	

