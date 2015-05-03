//! @file location.c
//! @brief Location
//! @author Atlantronic
#define WEAK_LOCATION
#include "location.h"
#include "kernel/module.h"
#include "kernel/portmacro.h"
#include "kernel/driver/usb.h"
#include "kernel/kinematics_model/kinematics_model.h"
#include "kernel/log.h"

static void location_cmd_set_position(void* arg);
VectPlan location_pos;
VectPlan location_speed;

static int location_module_init()
{
	usb_add_cmd(USB_CMD_LOCATION_SET_POSITION, location_cmd_set_position);

	return 0;
};

module_init(location_module_init, INIT_LOCATION);

void location_update(double voie_inv, Kinematics* kinematics_mes, float dt)
{
	VectPlan speed = kinematics_model_compute_speed(voie_inv, kinematics_mes);

	portENTER_CRITICAL();
	location_speed = speed;
	location_pos = location_pos + (loc_to_abs_speed(location_pos.theta, location_speed) * dt);
	portEXIT_CRITICAL();
}

VectPlan location_get_position()
{
	VectPlan p;
	portENTER_CRITICAL();
	p = location_pos;
	portEXIT_CRITICAL();
	return p;
}

VectPlan location_get_speed()
{
	VectPlan p;
	portENTER_CRITICAL();
	p = location_speed;
	portEXIT_CRITICAL();
	return p;
}

void location_set_position(VectPlan pos)
{
	log_format(LOG_INFO, "set position %d %d %d", (int)pos.x, (int)pos.y, (int)(pos.theta * 180 / M_PI));
	portENTER_CRITICAL();
	location_pos = pos;
	portEXIT_CRITICAL();
}

static void location_cmd_set_position(void* arg)
{
	VectPlan* pos = (VectPlan*) arg;
	location_set_position(*pos);
}
