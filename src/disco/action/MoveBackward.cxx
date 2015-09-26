#include "MoveBackward.h"

#include "kernel/log.h"
#include "middleware/motion/trajectory.h"
#include "kernel/location/location.h"

MoveBackward::MoveBackward(VectPlan firstcheckpoint, const char * name):Action(firstcheckpoint, name)
{

}

////////////////////////////////////////////////
/// function    : do_action()
/// descrition  : execute the action
/// param       : none
/// retrun      : -1 if fail or 0 if sucess
////////////////////////////////////////////////
int MoveBackward::do_action()
{
	Action::do_action();

	do
	{
	    trajectory_goto_near_xy(m_firstcheckpoint.x, m_firstcheckpoint.y, 0, WAY_ANY, AVOIDANCE_STOP);
	
		vTaskDelay(100);
	}while( trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) != 0); 
	return 0;
}
