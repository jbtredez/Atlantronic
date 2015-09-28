#include "SpotLight.h"
#include "kernel/log.h"
#include "middleware/trajectory/Trajectory.h"
#include "elevator.h"
#include "kernel/driver/dynamixel.h"
#include "disco/finger.h"
#include "kernel/location/location.h"

SpotLight::SpotLight(VectPlan firstcheckpoint,char * name, RobotState * elevator):ActionComposite(firstcheckpoint, name)
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
int SpotLight::do_action()
{
	Eelevator_state elevator_state = m_elevator->getelevatorstate();
	int result = 0;
	int nbelement =  m_elevator->getnumberelement();
	VectPlan position = location_get_position();
	Action * p_action;

	Action::do_action();

	switch(elevator_state)
	{
		//Cas où l'acsenceur est vide
		case ELEVATOR_EMPTY:
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
		//cas ou il faut récupérer des pieds ou déposer le spotlight
		case ELEVATOR_LIGHT:
			//Normal pas de break;
		case ELEVATOR_FEET:
			if(nbelement < MAX_ELEMENT )
			{
				p_action = find_action_not_done(ACTION_FEET,position);
				if(p_action != 0)
				{
					return p_action->do_action();
				}
			}
			p_action = find_action_not_done(ACTION_DROPZONE,position);
			if(p_action != 0)
			{
				result =  p_action->do_action();
				if( result != 0)
				{
					m_try ++;
				}
				else
				{
					m_try = -1;
					return result;
				}
			}
			break;
		///Dans les cas d'échec
		case ELEVATOR_GOBELET:
		default :
			log_format(LOG_INFO , "Action présente non utilisée");
			break;
	}

	return -1;	
}
