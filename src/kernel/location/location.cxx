//! @file location.c
//! @brief Location
//! @author Atlantronic

#include "location.h"
#include "kernel/module.h"
#include "kernel/portmacro.h"
#include "kernel/driver/usb.h"
#include "kernel/geometric_model/geometric_model.h"

static void location_cmd_set_position(void* arg);
VectPlan location_pos;
VectPlan location_speed;

static int location_module_init()
{
	usb_add_cmd(USB_CMD_LOCATION_SET_POSITION, location_cmd_set_position);

	// position initiale
	location_pos.x = 0;
	location_pos.y = 700;
	location_pos.theta = -M_PI/2;

	return 0;
};

module_init(location_module_init, INIT_LOCATION);

void location_update(Kinematics* kinematics_mes, int motorNum, float dt)
{
	(void) motorNum;
	float slippageSpeed;
	VectPlan speed = geometric_model_compute_speed(kinematics_mes, &slippageSpeed);

	portENTER_CRITICAL();
	location_speed = speed;
	location_pos = location_pos + dt * loc_to_abs_speed(location_pos.theta, location_speed);
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
	portENTER_CRITICAL();
	location_pos = pos;
	portEXIT_CRITICAL();
}

static void location_cmd_set_position(void* arg)
{
	VectPlan* pos = (VectPlan*) arg;
	location_set_position(*pos);
}
