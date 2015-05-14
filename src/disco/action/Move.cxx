

#include "kernel/log.h"
#include "kernel/motion/trajectory.h"
#include "disco/robot_state.h"
#include "elevator.h"
#include "kernel/driver/dynamixel.h"
#include "disco/finger.h"
#include "kernel/location/location.h"
#include "disco/action/Move.h"

#include "kernel/stratege_machine/action.h"
#include "kernel/math/vect_plan.h"
#define MOVE_APPROX_DIST 100
Move::Move(VectPlan firstcheckpoint,const char  * name):Action(firstcheckpoint, name)
{
	set_actiontype(ACTION_MOVE);
}

////////////////////////////////////////////////
/// function    : do_action()
/// descrition  : execute the action
/// param       : none
/// retrun      : -1 if fail or 0 if sucess
////////////////////////////////////////////////
int Move::do_action()
{

		do 
	{
		vTaskDelay(100);
		trajectory_goto_near(m_firstcheckpoint, MOVE_APPROX_DIST, WAY_ANY, AVOIDANCE_STOP) ;

	}while(  trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) != 0) ;
	return 0;

}
	
	

	
	

