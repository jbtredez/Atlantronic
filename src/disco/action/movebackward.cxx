#include "kernel/log.h"
#include "kernel/motion/trajectory.h"
#include "kernel/location/location.h"
#include "disco/action/movebackward.h"

#include "kernel/stratege_machine/action.h"
#include "kernel/math/vect_plan.h"

#define LIGHT_APPROX_DIST       100


movebackward::movebackward(VectPlan firstcheckpoint, const char * name):Action(firstcheckpoint, name)
{

}

////////////////////////////////////////////////
/// function    : do_action()
/// descrition  : execute the action
/// param       : none
/// retrun      : -1 if fail or 0 if sucess
////////////////////////////////////////////////
int movebackward::do_action()
{

	Action::do_action();

	do
	{
	    trajectory_goto(m_firstcheckpoint, WAY_BACKWARD, AVOIDANCE_STOP);
	
	}while( trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) != 0); 
	return 0;
}
	
	
