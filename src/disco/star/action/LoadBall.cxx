/*
 * LoadBall.cpp
 *
 *  Created on: 21 mai 2017
 *      Author: jul
 */

#include "LoadBall.h"


#include "kernel/log.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/match.h"
#include "disco/star/star.h"
#include "disco/star/servos.h"


/*Cratere 1
	m_firstcheckpoint.x = 765;
	m_firstcheckpoint.y = -460;
	m_firstcheckpoint.theta = -M_PI_2;

*/
/*Cratere 2
	m_firstcheckpoint.x = 2645;
	m_firstcheckpoint.y = 1645;
	m_firstcheckpoint.theta = -M_PI_2;

*/


LoadBall::LoadBall(VectPlan firstcheckpoint, uint32_t checkpoint, const char * name, RobotState * robot):
		Action(firstcheckpoint, name,(void*) robot)
{
	// TODO Auto-generated constructor stub

}

LoadBall::~LoadBall()
{
	// TODO Auto-generated destructor stub
}


int LoadBall::do_action()
{
	uint32_t actionResult = 0;
	Action::do_action();
	vTaskDelay(100);

	trajectory.goTo(m_firstcheckpoint, WAY_FORWARD, AVOIDANCE_STOP);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) != 0 )
	{

	}

	trajectory.straight(10);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) != 0 )
	{

	}
	//Activation de la turbine

	//Les balle sont aspirée
	vTaskDelay(1000);
	Servos::setNetState(NET_TRAP_CLOSE);

	vTaskDelay(1000);
	//Coupure de la turbine


	//On indique que l'on a des éléments dans le piège
	BemptyTrap = 0;

	m_state = ACTION_DONE;


	return actionResult;
}
 bool LoadBall::Ready()
{

	 //Si dans le piège il y a des balles la alors on n'execute pas l'action
	 return (BemptyTrap  && m_state != ACTION_DONE);
}


void LoadBall::Exit()
{

}
