/*
 * LoadBall.cpp
 *
 *  Created on: 21 mai 2017
 *      Author: jul
 */

#include "UnloadBall.h"


#include "kernel/log.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/match.h"
#include "disco/star/star.h"
#include "disco/star/servos.h"


/*Point d'entree
	m_firstcheckpoint.x = 2640;
	m_firstcheckpoint.y = 0;
	m_firstcheckpoint.theta = -M_PI_2;

*/

/*Point intermédiare
	m_firstcheckpoint.x = 1140;
	m_firstcheckpoint.y = -460;
	m_firstcheckpoint.theta = -M_PI_2;

*/



UnloadBall::UnloadBall(VectPlan firstcheckpoint, uint32_t checkpoint, const char * name, RobotState * robot):
		Action(firstcheckpoint, name,(void*) robot)
{
	// Angle à revoir
	m_firstcheckpoint.x = 1140;
	m_firstcheckpoint.y = 0;
	m_firstcheckpoint.theta = -M_PI_2;

	// Angle à revoir
	m_Intermedary_checkpoint.x = 1140;
	m_Intermedary_checkpoint.y = -460;
	m_Intermedary_checkpoint.theta = -M_PI_2;


}

UnloadBall::~UnloadBall()
{
	// TODO Auto-generated destructor stub
}

 void UnloadBall::Initialise(int stratcolor)
 {
	 Action::Initialise(stratcolor);
	 m_Intermedary_checkpoint.symetric(stratcolor);
 }

int UnloadBall::do_action()
{
	uint32_t actionResult = 0;
	Action::do_action();
	vTaskDelay(100);

	//Deplacement vers le pt d'insertion
	trajectory.goTo(m_firstcheckpoint, WAY_FORWARD, AVOIDANCE_STOP);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) != 0 )
	{

	}


	//Déplacement vers la bascule

	trajectory.goTo(m_firstcheckpoint, WAY_FORWARD, AVOIDANCE_STOP);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) != 0 )
	{

	}




	Servos::setNetState(NET_TRAP_OPEN);

	//On indique que l'on n'a pas des éléments dans le piège
	BemptyTrap = 1;



	return actionResult;
}
 bool UnloadBall::Ready()
{

	 //Si dans le piège il y a des balles la alors on execute l'action
	 return !BemptyTrap  ;
}


void UnloadBall::Exit()
{

}
