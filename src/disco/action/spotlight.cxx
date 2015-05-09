

#include "kernel/log.h"
#include "kernel/motion/trajectory.h"
#include "disco/robot_state.h"
#include "elevator.h"
#include "kernel/driver/dynamixel.h"
#include "disco/finger.h"
#include "kernel/location/location.h"
#include "disco/action/spotlight.h"

#include "kernel/stratege_machine/action.h"
#include "kernel/math/vect_plan.h"

spotlight::spotlight(VectPlan firstcheckpoint,robotstate * elevator):actioncomposite(firstcheckpoint)
{
	if(elevator != 0)
	{
		m_elevator =  elevator;
	}
	
	set_actiontype(ACTION_SPOTLIGHT);
}

////////////////////////////////////////////////
/// function    : do_action()
/// descrition  : execute the action
/// param       : none
/// retrun      : -1 if fail or 0 if sucess
////////////////////////////////////////////////
int spotlight::do_action()
{
	Eelevator_state elevator_state = m_elevator->getelevatorstate();
	int nbelement =  m_elevator->getnumberelement();
	VectPlan position;
	action * p_action;
	switch(elevator_state)
	{
		//Cas où l'acsenceur est vide
		case ELEVATOR_EMPTY:
		{
			p_action = find_action_not_done(ACTION_LIGHT,position);
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
		//cas ou il faut récupérer des pieds ou déposer le spotlight
		case ELEVATOR_LIGHT:
			//Normal pas de break;
		case ELEVATOR_FEET:
		{
			if(nbelement < MAX_ELEMENT )
			{
				p_action = find_action_not_done(ACTION_LIGHT,position);
				if(p_action != 0)
				{
					return p_action->do_action();
				}
				else
				break;
			}
			else
			{
				p_action = find_action_not_done(ACTION_LIGHT,position);
				if(p_action != 0)
				{
					return p_action->do_action();
				}
				break;
			}

		}
			
		///Dans les cas d'échec
		case ELEVATOR_GOBELET:
		default :
			log_format(LOG_INFO , "Action présente non utilisée");

	}

	return -1;	

}
	
	

	
	

