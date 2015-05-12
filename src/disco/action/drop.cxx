

#include "kernel/log.h"
#include "kernel/motion/trajectory.h"
#include "disco/robot_state.h"
#include "elevator.h"

#include "disco/finger.h"
#include "kernel/location/location.h"
#include "disco/action/drop.h"

#include "kernel/stratege_machine/action.h"
#include "kernel/math/vect_plan.h"


drop::drop(VectPlan firstcheckpoint,robotstate * elevator):action(firstcheckpoint)
{
	if(elevator != 0)
	{
		m_elevator = elevator;	
	}
	m_actiontype = ACTION_DROP;
}

////////////////////////////////////////////////
/// function    : do_action()
/// descrition  : execute the action
/// param       : none
/// retrun      : -1 if fail or 0 if sucess
////////////////////////////////////////////////
int drop::do_action()
{
	int result = -1;
	int essai = 0;

	if(m_elevator != 0)
	{
		m_try++;
		return -1;
	}


	Eelevator_state state = m_elevator->getelevatorstate();
	VectPlan drop = m_firstcheckpoint;
	//Si la reserve est vide
	if( ( state == ELEVATOR_EMPTY)   )
	{
		m_try++;
		return -1;
	}
	
	//On va dans à la position demandée
	do 
	{
		trajectory_goto_near(drop, DROP_APPROX_DIST, WAY_FORWARD, AVOIDANCE_STOP) ;
		
		if (   trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) == 0)
		{
			result = 0;
		}

		essai++;

		if(essai == 3)
		{
			m_try++;
			return -1;
		}
	} while(  result ==-1 ) ;

	// faut pas monter l'ascenseur au dela de 100 avec la pince du haut ouverte (sinon, ça frotte sur l'étage du milieu).
	elevator_set_position(100);

	vTaskDelay(100);
	finger_set_pos(FINGER_OPEN, FINGER_OPEN);
	vTaskDelay(400);
	
	m_elevator->setnumberelement(0);
 	m_elevator->setelevatorstate(ELEVATOR_EMPTY);
	m_try = -1;
	return result;
}
	
	
