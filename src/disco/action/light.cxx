

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

light::light(VectPlan firstcheckpoint, const char * name, robotstate * elevator):Action(firstcheckpoint, name)
{
	if(elevator != 0)
	{
		m_elevator =  elevator;
	}
	
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
		trajectory_goto_near(m_firstcheckpoint, LIGHT_APPROX_DIST, WAY_ANY, AVOIDANCE_STOP) ;
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

	finger_set_pos(FINGER_CLOSE, FINGER_HALF_CLOSE);
	vTaskDelay(500);
	//Vérifie si on a attrapé quelque chose
	// TODO a voir plus tard, ne marche pas (trop long)
	//if( ax12.isFlagActive(AX12_LOW_FINGER, DYNAMIXEL_FLAG_STUCK) )
	{
		log(LOG_INFO, "light on board");
		m_elevator->setnumberelement(nbelement + 1);
	 	m_elevator->setelevatorstate(ELEVATOR_LIGHT);
		m_try = -1;
		finger_set_pos(FINGER_HALF_CLOSE, FINGER_HALF_CLOSE);
		return 0;
	}

/*	log(LOG_INFO, "light lost");
	m_try++;	
	return -1;*/
}
	
	

