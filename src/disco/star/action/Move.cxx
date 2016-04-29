#include "kernel/log.h"
#include "middleware/trajectory/Trajectory.h"
#include "disco/star/robot_state.h"
#include "kernel/driver/Dynamixel.h"
#include "kernel/location/location.h"
#include "Move.h"
#include "disco/star/star.h"

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
		trajectory.goToNear(m_firstcheckpoint, 0, WAY_FORWARD, AVOIDANCE_STOP) ;

	}while( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) != 0) ;
	return 0;
}
