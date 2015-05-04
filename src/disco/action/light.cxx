

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

#define LIGHT_APPROX_DIST       100
light::light(VectPlan firstcheckpoint,robotstate * elevator):action(firstcheckpoint)
{
m_elevator = elevator;
	 
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
	
	//Si la reserve est pleine on quitte la fonction
	if(m_elevator->getnumberelement() >= MAX_ELEMENT)
	{
		return -1;
	}

		

	elevator_set_position(100);
	vTaskDelay(100);
	finger_set_pos(FINGER_OPEN, FINGER_OPEN);

	do 
	{
		trajectory_goto_near(m_firstcheckpoint, LIGHT_APPROX_DIST, WAY_FORWARD, AVOIDANCE_STOP) ;
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


	vTaskDelay(500);
	elevator_set_position(0);
	vTaskDelay(800);
	finger_set_pos(FINGER_CLOSE, FINGER_CLOSE);
	vTaskDelay(300);
	//Vérifie si on a attrapé quelque chose
	if(ax12.isFlagActive(AX12_LOW_FINGER, DYNAMIXEL_FLAG_STUCK) || ax12.isFlagActive(AX12_HIGH_FINGER, DYNAMIXEL_FLAG_STUCK))
	{
		
		m_elevator->setnumberelement(nbelement + 1);
	 	m_elevator->setelevatorstate(ELEVATOR_LIGHT);
		return 0;
	}	
	return -1;
}
	
	

