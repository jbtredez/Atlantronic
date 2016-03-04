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
#include "disco/wing.h"
#include "disco/elevator.h"
#include "disco/finger.h"
#include "disco/star.h"

void recalage()
{
	VectPlan pos(1200, 0, M_PI_2);
	VectPlan posInit(1050, 0, -3*M_PI/4);
	VectPlan firstcheckpoint(630, -355, 0);
	posInit.theta = atan2f(firstcheckpoint.y - posInit.y, firstcheckpoint.x - posInit.x);

	int color = match_get_color();

	log(LOG_INFO, "recalage...");

	location.setPosition(pos.symetric(color));
	setTableColor(color);

// TODO coder recalage 2016
return;
	wing_set_position(WING_PARK, WING_PARK);
	elevator_set_position(85);
	finger_set_pos(FINGER_CLOSE, FINGER_HALF_OPEN);
	finger_bottom_set_pos(FINGER_BOTTOM_OPEN, FINGER_BOTTOM_OPEN);
	vTaskDelay(500);
	finger_set_pos(FINGER_CLOSE, FINGER_CLOSE);

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
	trajectory.straight(200);

	if( trajectory.wait(TRAJECTORY_STATE_COLISION, 10000) )
	{
		goto free;
	}

	pos = location.getPosition();
	pos.y = 200 - PARAM_LEFT_CORNER_X;
	pos.theta = M_PI_2;
	location.setPosition(pos);

	// on doit attendre au moins un cycle de la tache control
	// pour la prise en compte de la nouvelle position
	vTaskDelay(ms_to_tick(100));

	trajectory.straight(-200 + PARAM_LEFT_CORNER_X);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) )
	{
		goto free;
	}

	if( color == COLOR_GREEN )
	{
		trajectory.rotateTo(0);
	}
	else
	{
		trajectory.rotateTo(M_PI);
	}
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) )
	{
		goto free;
	}

	trajectory.straight(200);
	if( trajectory.wait(TRAJECTORY_STATE_COLISION, 10000) )
	{
		goto free;
	}

	pos = location.getPosition();
	pos.x = 1430 - PARAM_LEFT_CORNER_X;
	pos.theta = 0;
	location.setPosition(pos.symetric(color));

	// on doit attendre au moins un cycle de la tache control
	// pour la prise en compte de la nouvelle position
	vTaskDelay(ms_to_tick(100));
/*
	trajectory_straight(-100);
	if( trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) )
	{
		goto free;
	}

	vTaskDelay(200);*/

	trajectory.goToNear(posInit.symetric(color), 0, WAY_ANY, AVOIDANCE_STOP);
	if( trajectory.wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) )
	{
		goto free;
	}

	vTaskDelay(200);
	finger_set_pos(FINGER_CLOSE, FINGER_HALF_OPEN);
	vTaskDelay(300);
	finger_set_pos(FINGER_HALF_OPEN, FINGER_HALF_OPEN);
	elevator_set_position(0);
	log(LOG_INFO, "recalage termine");

free:
	trajectory.setKinematicsParam(linParamOrig, angParamOrig);
	//	trajectory.trajectory_free();
	trajectory.enableHokuyo(true);
	trajectory.enableStaticCheck(true);
	motion.enableAntico(true);
}
