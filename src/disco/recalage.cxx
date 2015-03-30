#define WEAK_RECALAGE
#include "recalage.h"
#include "kernel/module.h"
#include "kernel/systick.h"
#include "kernel/rcc.h"
#include "kernel/log.h"
#include "kernel/robot_parameters.h"
#include "kernel/driver/usb.h"
#include "kernel/location/location.h"
#include "kernel/match.h"
#include "kernel/motion/trajectory.h"

#include "disco/wing.h"
#include "disco/elevator.h"
#include "disco/finger.h"

void recalage()
{
	VectPlan pos(1200, 0, M_PI_2);
	int color = match_get_color();

	log(LOG_INFO, "recalage...");

	location_set_position(pos.symetric(color));

	wing_set_position(WING_PARK, WING_PARK);
	elevator_set_position(100);
	finger_set_pos(FINGER_HALF_CLOSE, FINGER_HALF_CLOSE);
	// TODO ranger porte tapis

/*	KinematicsParameters linParam = {1000, 1500, 1500};
	KinematicsParameters angParam = {1, 1, 1};

	trajectory_set_kinematics_param();*/

	trajectory_disable_hokuyo();
	trajectory_disable_static_check();

	motion_enable(true);
	trajectory_straight(500);

	if( trajectory_wait(TRAJECTORY_STATE_COLISION, 10000) )
	{
		goto free;
	}

	pos = location_get_position();
	pos.y = 200 - PARAM_LEFT_CORNER_X;
	pos.theta = M_PI_2;
	location_set_position(pos);

	// on doit attendre au moins un cycle de la tache control
	// pour la prise en compte de la nouvelle position
	vTaskDelay(ms_to_tick(20));

	trajectory_straight(-75);
	if( trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) )
	{
		goto free;
	}

	if( color == COLOR_GREEN )
	{
		trajectory_rotate_to(0);
	}
	else
	{
		trajectory_rotate_to(M_PI);
	}
	if( trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) )
	{
		goto free;
	}

	trajectory_straight(500);
	if( trajectory_wait(TRAJECTORY_STATE_COLISION, 10000) )
	{
		goto free;
	}

	pos = location_get_position();
	pos.x = 1430 - PARAM_LEFT_CORNER_X;
	pos.theta = 0;
	location_set_position(pos.symetric(color));

	// on doit attendre au moins un cycle de la tache control
	// pour la prise en compte de la nouvelle position
	vTaskDelay(ms_to_tick(20));

	trajectory_straight(-20);
	if( trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) )
	{
		goto free;
	}

	vTaskDelay(ms_to_tick(200));
	log(LOG_INFO, "recalage termine");

free:
//	trajectory_free();
	trajectory_enable_hokuyo();
	trajectory_enable_static_check();
}
