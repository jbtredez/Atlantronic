

#include "kernel/log.h"
#include "kernel/motion/trajectory.h"
#include "disco/robot_state.h"
#include "elevator.h"
#include "kernel/driver/dynamixel.h"
#include "disco/finger.h"
#include "kernel/location/location.h"
#include "disco/action/feet_lateral.h"

#include "kernel/stratege_machine/action.h"
#include "kernel/math/vect_plan.h"


FeetLateral::FeetLateral(VectPlan firstcheckpoint, const char * name, robotstate * elevator):Action(firstcheckpoint, name)
{
	if(elevator != 0)
	{
		m_elevator = elevator;	
	}

	m_actiontype = ACTION_FEET;
	p1 = VectPlan(-1005, 604, 0);
	p2 = VectPlan(-758, 735, 0);
}

////////////////////////////////////////////////
/// function    : do_action()
/// descrition  : execute the action
/// param       : none
/// retrun      : -1 if fail or 0 if sucess
////////////////////////////////////////////////
int FeetLateral::do_action()
{
	int result = -1;
	int essai = 0;
	Action::do_action();
	if(m_elevator == 0)
	{
		return -1;
	}

	int nbelement = m_elevator->getnumberelement();

	//S ila reserve est pleine on quitte la fonction
	if(nbelement >= MAX_ELEMENT)
	{
		return -1;
	}

	//On serre les élément 
	finger_set_pos(FINGER_CLOSE, FINGER_HALF_OPEN);
	vTaskDelay(500);

	//On souleve la pile pour ajouter le nouvel element
	elevator_set_position(85);

	do 
	{
		trajectory_goto_near_xy(p1.x,p1.y, 0, WAY_FORWARD, AVOIDANCE_STOP) ;

		if (   trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) == 0)
		{
			result = 0;
		}
		essai++;
		if(essai == 3)
		{
			elevator_set_position(0);

			m_try++;
			return -1;
		}
		vTaskDelay(100);

	} while(  result ==-1 ) ;

	do
	{
		trajectory_goto_near_xy(p2.x,p2.y, 0, WAY_FORWARD, AVOIDANCE_STOP) ;

		if (   trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) == 0)
		{
			result = 0;
		}
		essai++;
		if(essai == 3)
		{
			elevator_set_position(0);

			m_try++;
			return -1;
		}
		vTaskDelay(100);

	} while(  result ==-1 ) ;

	//finger_bottom_set_pos(FINGER_BOTTOM_CLOSE, FINGER_BOTTOM_CLOSE);
	finger_set_pos(FINGER_HALF_CLOSE, FINGER_HALF_OPEN);

	vTaskDelay(500);

	finger_bottom_set_pos(FINGER_BOTTOM_OPEN, FINGER_BOTTOM_OPEN);
	finger_set_pos(FINGER_OPEN, FINGER_OPEN);
	elevator_set_position(0);
	vTaskDelay(800);


	finger_set_pos(FINGER_HALF_CLOSE, FINGER_HALF_OPEN);
	vTaskDelay(200);
	
	//Vérifie si on a attrapé quelque chose
	//if(elevator_omron_active())
	//{
		m_elevator->setnumberelement(nbelement + 1);
	 	m_elevator->setelevatorstate(ELEVATOR_FEET);

		//(FINGER_HALF_CLOSE). Cela ne serre pas le pied et le pied reste bloqué. On ne solicite pas non plus
		finger_set_pos(FINGER_HALF_CLOSE, FINGER_HALF_OPEN);
		m_try = -1;

		return 0;
	//}	

	//m_try++;
	//return -1;
}
	
	

