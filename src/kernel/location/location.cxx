//! @file location.c
//! @brief Location
//! @author Atlantronic

#include "location.h"
#include "kernel/module.h"
#include "kernel/portmacro.h"
#include "kernel/driver/usb.h"


static void location_cmd_set_position(void* arg);
VectPlan location_pos;

static int location_module_init()
{
	usb_add_cmd(USB_CMD_LOCATION_SET_POSITION, location_cmd_set_position);

	return 0;
};

module_init(location_module_init, INIT_LOCATION);
/*
void location_update()
{
	odometry_update();

	// TODO fusion de données odometrie, balises et capteurs / cases
	// en attendant : odometrie seule
	portENTER_CRITICAL();
	location_kinematics = odometry_get_kinematics();
	portEXIT_CRITICAL();
}
*/
VectPlan location_get_position()
{
	VectPlan p;
	portENTER_CRITICAL();
	p = location_pos;
	portEXIT_CRITICAL();
	return p;
}
/*
struct kinematics location_get_kinematics()
{
	struct kinematics k;
	portENTER_CRITICAL();
	k = location_kinematics;
	portEXIT_CRITICAL();
	return k;
}
*/

void location_set_position(VectPlan pos)
{
	portENTER_CRITICAL();
	location_pos = pos;
	portEXIT_CRITICAL();
}

static void location_cmd_set_position(void* arg)
{
	struct location_cmd_arg* cmd_arg = (struct location_cmd_arg*) arg;
	VectPlan pos(cmd_arg->x, cmd_arg->y, cmd_arg->theta);

	location_set_position(pos);
}
