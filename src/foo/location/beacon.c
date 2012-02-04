//! @file beacon.c
//! @brief Beacon
//! @author Atlantronic

#include "location/beacon.h"
#include "kernel/module.h"
#include "kernel/robot_parameters.h"
#include "kernel/portmacro.h"

static struct fx_vect_pos beacon_pos;

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

struct fx_vect_pos beacon_get_position()
{
	struct fx_vect_pos p;
	portENTER_CRITICAL();
	p = beacon_pos;
	portEXIT_CRITICAL();
	return p;
}
