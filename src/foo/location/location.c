//! @file location.c
//! @brief Location
//! @author Atlantronic

#include "location/location.h"
#include "kernel/module.h"
#include "kernel/portmacro.h"
#include <math.h>

static float location_v_distance;   //!< en "m / unité de temps"
static float location_v_rotate;     //!< en "rd / unité de temps"

static struct vect_pos location_pos;

static int location_module_init()
{
	location_pos = odometry_get_position();
	location_v_distance = odometry_get_speed_curv_abs();
	location_v_rotate = odometry_get_speed_rot();

	return 0;
};

module_init(location_module_init, INIT_LOCATION);

void location_update()
{
	odometry_update();

	// TODO fusion de données odometrie, balises et capteurs / cases
	// en attendant : odometrie seule
	portENTER_CRITICAL();
	location_pos = odometry_get_position();
	location_v_distance = odometry_get_speed_curv_abs();
	location_v_rotate = odometry_get_speed_rot();
	portEXIT_CRITICAL();
}

struct vect_pos location_get_position()
{
	struct vect_pos p;
	portENTER_CRITICAL();
	p = location_pos;
	portEXIT_CRITICAL();
	return p;
}

void location_set_position(float x, float y, float alpha)
{
	portENTER_CRITICAL();
	location_pos.x = x;
	location_pos.y = y;
	location_pos.alpha = alpha;
	location_pos.ca = cos(alpha);
	location_pos.sa = sin(alpha);
	odometry_set_position(location_pos);
	portEXIT_CRITICAL();
}

float location_get_speed_curv_abs()
{
	float v;
	portENTER_CRITICAL();
	v = location_v_distance;
	portEXIT_CRITICAL();
	return v;
}

float location_get_speed_rot()
{
	float v;
	portENTER_CRITICAL();
	v = location_v_rotate;
	portEXIT_CRITICAL();
	return v;
}
