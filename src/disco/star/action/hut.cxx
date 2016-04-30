#include "kernel/log.h"
#include "middleware/trajectory/Trajectory.h"
#include "kernel/match.h"
#include "hut.h"
#include "disco/star/star.h"


Hut::Hut(VectPlan firstcheckpoint, const char * name, RobotState * robot):Action(firstcheckpoint, name)
{
	if(robot != 0)
	{
		m_robot =  robot;
	}
	
	m_actiontype = ACTION_HUT;
}

void Hut::Initialise(int stratcolor)
{
	Action::Initialise(stratcolor);
	this->stratColor = stratcolor;
}

////////////////////////////////////////////////
/// function    : do_action()
/// descrition  : execute the action
/// param       : none
/// retrun      : -1 if fail or 0 if success
////////////////////////////////////////////////
int Hut::do_action()
{
	int bresult = 0;

	VectPlan nextToHut1;
	VectPlan flagExt(1200, 700, -3.14f/2);
	VectPlan flagInt(900, 700, -3.14f/2);
	Action::do_action();
	flagInt = flagInt.symetric(stratColor);
	flagExt = flagExt.symetric(stratColor);

	if(m_retry < 0 )
	{
		return 0;
	}

	// On s'approche de la première cabane
	trajectory.goTo(flagInt, WAY_BACKWARD, AVOIDANCE_STOP) ;
	if(trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 15000) != 0)
	{
		return -1;
	}
	vTaskDelay(100);



	// On ferme la porte
	bresult = goToWall();

	if ( ! bresult)
	{
		// On s'approche de la deuxième cabane
		do
		{
			trajectory.goTo(flagExt, WAY_BACKWARD, AVOIDANCE_STOP) ;
			vTaskDelay(100);
		}while( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) != 0) ;

		// On ferme la porte
		bresult = goToWall();
	}

	return bresult;
}


int Hut::goToWall(void)
{
	int bresult = 0;
	// On ralentit le robot pour la collision
	KinematicsParameters linParamOrig;
	KinematicsParameters angParamOrig;
	trajectory.getKinematicsParam(&linParamOrig, &angParamOrig);

	KinematicsParameters linParam = {100, 300, 300};
	KinematicsParameters angParam = angParamOrig;
	angParam.vMax /= 2;
	angParam.aMax /= 2;
	angParam.dMax /= 2;

	trajectory.setKinematicsParam(linParam, angParam);

	trajectory.enableHokuyo(false);
	trajectory.enableStaticCheck(false);
	motion.enableAntico(false);

	motion.enable(true);
	vTaskDelay(300);
	trajectory.straight(-400);

	if( trajectory.wait(TRAJECTORY_STATE_COLISION, 4000) != 0)
	{
		bresult = 1;
	}

	trajectory.enableHokuyo(true);
	trajectory.enableStaticCheck(true);
	motion.enableAntico(true);
	trajectory.setKinematicsParam(linParamOrig, angParamOrig);

	vTaskDelay(300);
	trajectory.straight(200);
	trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 4000);
	vTaskDelay(100);

	return bresult;
}
