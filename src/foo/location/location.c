//! @file location.c
//! @brief Location
//! @author Atlantronic

#include "location/location.h"
#include "kernel/module.h"
#include "kernel/portmacro.h"
#include "kernel/math/trigo.h"
#include "kernel/driver/can.h"
#include "kernel/can/can_id.h"
#include "kernel/driver/usb.h"

static struct kinematics location_kinematics;
static void location_cmd_set_position(void* arg);

static int location_module_init()
{
	location_kinematics = odometry_get_kinematics();

	usb_add_cmd(USB_CMD_LOCATION_SET_POSITION, location_cmd_set_position);

	return 0;
};

module_init(location_module_init, INIT_LOCATION);

void location_update()
{
	odometry_update();

	// TODO fusion de données odometrie, balises et capteurs / cases
	// en attendant : odometrie seule
	portENTER_CRITICAL();
	location_kinematics = odometry_get_kinematics();
	portEXIT_CRITICAL();
#if 0
	// envoi sur le can.
	// pas d'attente car on passe regulierement ici
	struct can_msg msg;
	msg.format = CAN_STANDARD_FORMAT;
	msg.type = CAN_DATA_FRAME;
	msg.size = 8;
	msg.id = CAN_KINEMATICS_1;
	msg._data.low = location_kinematics.x;
	msg._data.high = location_kinematics.y;
	can_write(&msg, 0);
	msg.id = CAN_KINEMATICS_2;
	msg._data.low = location_kinematics.alpha;
	msg._data.high = location_kinematics.v;
	can_write(&msg, 0);
	msg.id = CAN_KINEMATICS_3;
	msg.size = 4;
	msg._data.low = location_kinematics.w;
	can_write(&msg, 0);
#endif
}

struct fx_vect_pos location_get_position()
{
	struct fx_vect_pos p;
	portENTER_CRITICAL();
	p.x = location_kinematics.x;
	p.y = location_kinematics.y;
	p.alpha = location_kinematics.alpha;
	p.ca = location_kinematics.ca;
	p.sa = location_kinematics.sa;
	portEXIT_CRITICAL();
	return p;
}

struct kinematics location_get_kinematics()
{
	struct kinematics k;
	portENTER_CRITICAL();
	k = location_kinematics;
	portEXIT_CRITICAL();
	return k;
}

void location_set_position(int32_t x, int32_t y, int32_t alpha)
{
	portENTER_CRITICAL();
	odometry_set_position(x, y, alpha);
	location_kinematics = odometry_get_kinematics();
	portEXIT_CRITICAL();
}

static void location_cmd_set_position(void* arg)
{
	struct location_cmd_arg* cmd_arg = (struct location_cmd_arg*) arg;

	location_set_position(cmd_arg->x, cmd_arg->y, cmd_arg->alpha);
}
