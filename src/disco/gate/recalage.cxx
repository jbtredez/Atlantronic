#define WEAK_RECALAGE
#include "recalage.h"
#include "kernel/module.h"
#include "kernel/systick.h"
#include "kernel/rcc.h"
#include "kernel/log.h"
#include "kernel/driver/usb.h"
#include "kernel/location/location.h"
#include "kernel/match.h"
#include "middleware/trajectory/Trajectory.h"
#include "disco/table.h"
#include "disco/gate/gate.h"


#define STAR_RECALAGE_AVANT
#ifdef STAR_RECALAGE_AVANT
#define OPPOSED_ANGLE(x) ((x) - M_PI)
#define RECALAGE_WAY(x) (-1 * (x))
#else
#define OPPOSED_ANGLE(x) (x)
#define RECALAGE_WAY(x) (x)
#endif


void recalage()
{
	// position initiale estimee (endroit ou on pose le robot pour le recalage) couleur bleu
	int color = match_get_color();
	VectPlan pos(GATE_INIT_POS_X, GATE_INIT_POS_Y, GATE_INIT_POS_THETA);
	location.setPosition(pos.symetric(color));

	// VERSION 2017
	KinematicsParameters linParamOrig;
	KinematicsParameters angParamOrig;
	trajectory.getKinematicsParam(&linParamOrig, &angParamOrig);

	KinematicsParameters linParam = {100, 300, 300};
	KinematicsParameters angParam = angParamOrig;
	angParam.vMax /= 2;
	angParam.aMax /= 2;
	angParam.dMax /= 2;

	trajectory.setKinematicsParam(linParam, angParam);
	motion.enable(true);


	trajectory.straight(500);
	if( trajectory.wait(TRAJECTORY_STATE_COLISION, 5000) )
	{


	}

	vTaskDelay(500);

	pos.y = 1000-203;
	pos.theta = M_PI_2;
	location.setPosition(pos.symetric(color));


	trajectory.straight(-200);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) )
	{
		// Error?
	}

	vTaskDelay(500);

	float angle = (color == COLOR_YELLOW)? M_PI : -M_PI;

	trajectory.rotate(angle);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) )
	{
		// Error?
	}

	trajectory.straight(-300);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 5000) )
	{
		// Error?
	}

	trajectory.setKinematicsParam(linParamOrig, angParamOrig);


		// VERSION 2016
#if 0
	VectPlan posInit(1000, -600, OPPOSED_ANGLE(M_PI_2));
	VectPlan firstcheckpoint(0, -750, M_PI_2);
	//posInit.theta = atan2f(firstcheckpoint.y - posInit.y, firstcheckpoint.x - posInit.x);

	// Initialiser les ax 12

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

	KinematicsParameters linParam = {300, 600, 600};
	KinematicsParameters angParam = angParamOrig;
	angParam.vMax /= 2;
	angParam.aMax /= 2;
	angParam.dMax /= 2;

	trajectory.setKinematicsParam(linParam, angParam);

	trajectory.enableHokuyo(false);
	trajectory.enableStaticCheck(false);
	motion.enableAntico(false);

	motion.enable(true);
	trajectory.straight(RECALAGE_WAY(-1000));
	if( trajectory.wait(TRAJECTORY_STATE_COLISION, 20000) )
	{
		goto free;
	}

	pos = location.getPosition();
	pos.y = -1000 + GATE_HALF_LENGTH;
	pos.theta = OPPOSED_ANGLE(M_PI_2);
	location.setPosition(pos);

	// on doit attendre au moins un cycle de la tache control
	// pour la prise en compte de la nouvelle position
	vTaskDelay(ms_to_tick(100));

	trajectory.straight(RECALAGE_WAY(900 + 200 + 105));
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 50000) )
	{
		goto free;
	}

	vTaskDelay(500);
	if( color == COLOR_BLUE )
	{
		trajectory.rotateTo(OPPOSED_ANGLE(M_PI));
	}
	else
	{
		trajectory.rotateTo(OPPOSED_ANGLE(0));
	}
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 50000) )
	{
		goto free;
	}

	vTaskDelay(500);
	trajectory.straight(RECALAGE_WAY(-1300));
	if( trajectory.wait(TRAJECTORY_STATE_COLISION, 50000) )
	{
		goto free;
	}

	pos = location.getPosition();
	pos.x = 1500 - GATE_HALF_LENGTH;
	pos.theta = OPPOSED_ANGLE(M_PI);
	location.setPosition(pos.symetric(color));

	// on doit attendre au moins un cycle de la tache control
	// pour la prise en compte de la nouvelle position
	vTaskDelay(ms_to_tick(100));

	trajectory.straight(RECALAGE_WAY(75));
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 50000) )
	{
		goto free;
	}

	do
	{
		vTaskDelay(100);
		VectPlan postion;
		postion.x = 1315;
		postion.y = 9;
		postion.theta = OPPOSED_ANGLE(M_PI);


		//trajectory.goToNear(postion.symetric(color), 0, WAY_FORWARD, AVOIDANCE_STOP) ;

	}while( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 50000) != 0) ;

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
#endif
}

