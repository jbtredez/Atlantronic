//! @file beacon.c
//! @brief Beacon
//! @author Jean-Baptiste Trédez

#include "beacon.h"
#include "module.h"
#include "robot_parameters.h"
#include "portmacro.h"

static struct vect_pos beacon_pos;

static int beacon_module_init()
{
	beacon_pos.x = 0;
	beacon_pos.y = 0;
	beacon_pos.alpha = 0;

	return 0;
};

module_init(beacon_module_init, INIT_BEACON);

void beacon_update()
{
	#warning TODO beacon_update
}

struct vect_pos beacon_get_position()
{
	struct vect_pos p;
	portENTER_CRITICAL();
	p = beacon_pos;
	portEXIT_CRITICAL();
	return p;
}
