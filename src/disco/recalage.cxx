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

// TODO voir / bug simu
__OPTIMIZE_ZERO__ void recalage()
{
	VectPlan pos(1200, 0, M_PI_2);
	int color = match_get_color();
	pos.symetric(color);

	log(LOG_INFO, "recalage...");

	// TODO ranger les pince, ailes, ascenseur et porte tapis
	location_set_position(pos);

/*	KinematicsParameters linParam = {1000, 1500, 1500};
	KinematicsParameters angParam = {1, 1, 1};

	trajectory_set_kinematics_param();*/

	trajectory_disable_hokuyo();
	trajectory_disable_static_check();
	//trajectory_straight_to_wall();

	motion_enable(true);
	trajectory_straight(200);

	if( trajectory_wait(TRAJECTORY_STATE_COLISION, 10000) )
	{
// TODO bug detection collision
//		goto free;
	}

	pos = location_get_position();
	pos.y = 200 - PARAM_LEFT_CORNER_X;
	pos.theta = M_PI_2;
	location_set_position(pos);

	// on doit attendre au moins un cycle de la tache control
	// pour la prise en compte de la nouvelle position
	vTaskDelay(ms_to_tick(100));

	trajectory_straight(-75);
	if( trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) )
	{
		goto free;
	}

	vTaskDelay(ms_to_tick(100));
	/*if( color == COLOR_GREEN )
	{
		trajectory_rotate_to(M_PI);
	}
	else
	{
		trajectory_rotate_to(0);
	}*/
	pos = VectPlan(1200, 0, 0);
	pos.symetric(color);
	trajectory_goto(pos, WAY_FORWARD, AVOIDANCE_STOP);
	if( trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) )
	{
		goto free;
	}

#if 0
	trajectory_straight(-2000 << 16);
	if( trajectory_wait( TRAJECTORY_STATE_COLISION, 10000) )
	{
		goto free;
	}

	trajectory_straight_to_wall();
	if( trajectory_wait(TRAJECTORY_STATE_COLISION, 10000) )
	{
		goto free;
	}

	pos = location_get_position();
	if(getcolor() == COLOR_BLUE)
	{
		location_set_position(-((1500 << 16) + PARAM_NP_X), pos.y, 0);
	}
	else
	{
		location_set_position(((1500 << 16) + PARAM_NP_X), pos.y, 1 << 25);
	}

	// on doit attendre au moins un cycle de la tache control
	// pour la prise en compte de la nouvelle position
	vTaskDelay(ms_to_tick(10));

	trajectory_straight(200 << 16);
	if( trajectory_wait(TRAJECTORY_STATE_TARGET_REACHED, 10000) )
	{
		goto free;
	}



#endif

	vTaskDelay(ms_to_tick(200));
	log(LOG_INFO, "recalage termine");

free:
//	trajectory_free();
	trajectory_enable_hokuyo();
	trajectory_enable_static_check();;
}
