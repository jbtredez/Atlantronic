#include "MoveBackward.h"

#include "kernel/log.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/location/location.h"
#include "disco/star/star.h"

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
		trajectory.goToNearXy(m_firstcheckpoint.x, m_firstcheckpoint.y, 0, WAY_ANY, AVOIDANCE_STOP);
	
		vTaskDelay(100);
	}while( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) != 0);
	return 0;
}
