#define WEAK_RECALAGE
#include "recalage.h"
#include "kernel/module.h"
#include "kernel/systick.h"
#include "kernel/rcc.h"
#include "kernel/log.h"
#include "disco/robot_parameters.h"
#include "kernel/driver/usb.h"
#include "kernel/location/location.h"
#include "kernel/match.h"
#include "middleware/trajectory/Trajectory.h"
#include "disco/table.h"
#include "disco/gate/gate.h"

void recalage()
{
	VectPlan pos(1200, 0, 0);
	VectPlan posInit(1000, -600, -M_PI_2);
	VectPlan firstcheckpoint(0, -750, M_PI_2);
	//posInit.theta = atan2f(firstcheckpoint.y - posInit.y, firstcheckpoint.x - posInit.x);

	int color = match_get_color();

#if 1
	location.setPosition(posInit.symetric(color));
	setTableColor(color);
#else
	location.setPosition(pos.symetric(color));
	setTableColor(color);
	return;
#endif

	// Mettre tous les actionneurs en position de départ

	// Lancer calage x et tetha

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
	trajectory.straight(1000);
	if( trajectory.wait(TRAJECTORY_STATE_COLISION, 10000) )
	{
		goto free;
	}

	pos = location.getPosition();
	pos.y = -1000 + PARAM_LEFT_CORNER_Y;
	pos.theta = -M_PI_2;
	location.setPosition(pos);

	// on doit attendre au moins un cycle de la tache control
	// pour la prise en compte de la nouvelle position
	vTaskDelay(ms_to_tick(100));

//	trajectory.straight(-900);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 20000) )
	{
		goto free;
	}
/*
	vTaskDelay(500);
	if( color == COLOR_GREEN )
	{
		trajectory.rotateTo(M_PI);
	}
	else
	{
		trajectory.rotateTo(0);
	}
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) )
	{
		goto free;
	}

	vTaskDelay(500);
	trajectory.straight(-1000);
	if( trajectory.wait(TRAJECTORY_STATE_COLISION, 10000) )
	{
		goto free;
	}

	pos = location.getPosition();
	pos.x = 1500 - PARAM_LEFT_CORNER_X;
	pos.theta = M_PI;
	location.setPosition(pos.symetric(color));

	// on doit attendre au moins un cycle de la tache control
	// pour la prise en compte de la nouvelle position
	vTaskDelay(ms_to_tick(100));

	trajectory.straight(75);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) )
	{
		goto free;
	}
/*
	vTaskDelay(500);
	trajectory.rotateTo(0);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) )
	{
		goto free;
	}
	vTaskDelay(100);
	*/

free:
	trajectory.setKinematicsParam(linParamOrig, angParamOrig);
	trajectory.enableHokuyo(true);
	trajectory.enableStaticCheck(true);
	motion.enableAntico(true);
}
