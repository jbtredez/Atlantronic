//! @file location.c
//! @brief Location
//! @author Atlantronic

#include "location/location.h"
#include "kernel/module.h"
#include "kernel/portmacro.h"
#include "kernel/math/trigo.h"

static int32_t location_v_distance;   //!< en "m / unité de temps"
static int32_t location_v_rotate;     //!< en "rd / unité de temps"

static struct fx_vect_pos location_pos;

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

struct fx_vect_pos location_get_position()
{
	struct fx_vect_pos p;
	portENTER_CRITICAL();
	p = location_pos;
	portEXIT_CRITICAL();
	return p;
}

void location_set_position(int32_t x, int32_t y, int32_t alpha)
{
	portENTER_CRITICAL();
	location_pos.x = x;
	location_pos.y = y;
	location_pos.alpha = alpha;
	location_pos.ca = fx_cos(alpha);
	location_pos.sa = fx_sin(alpha);
	odometry_set_position(location_pos);
	portEXIT_CRITICAL();
}

int32_t location_get_speed_curv_abs()
{
	int32_t v;
	portENTER_CRITICAL();
	v = location_v_distance;
	portEXIT_CRITICAL();
	return v;
}

int32_t location_get_speed_rot()
{
	int32_t v;
	portENTER_CRITICAL();
	v = location_v_rotate;
	portEXIT_CRITICAL();
	return v;
}
