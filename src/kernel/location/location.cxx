//! @file location.c
//! @brief Location
//! @author Atlantronic
#define WEAK_LOCATION
#include "location.h"
#include "kernel/module.h"
#include "kernel/portmacro.h"
#include "kernel/driver/usb.h"
#include "kernel/log.h"

void Location::init(KinematicsModel* kinematicsModel)
{
	m_kinematicsModel = kinematicsModel;
	usb_add_cmd(USB_CMD_LOCATION_SET_POSITION, cmdSetPosition, this);
}

void Location::update(double voie_inv, Kinematics* kinematics_mes, float dt)
{
	VectPlan speed = m_kinematicsModel->computeSpeed(voie_inv, kinematics_mes);

	portENTER_CRITICAL();
	location_speed = speed;
	location_pos = location_pos + dt * loc_to_abs_speed(location_pos.theta, location_speed);
	portEXIT_CRITICAL();
}

VectPlan Location::getPosition()
{
	VectPlan p;
	portENTER_CRITICAL();
	p = location_pos;
	portEXIT_CRITICAL();
	return p;
}

VectPlan Location::getSpeed()
{
	VectPlan p;
	portENTER_CRITICAL();
	p = location_speed;
	portEXIT_CRITICAL();
	return p;
}

void Location::setPosition(VectPlan pos)
{
	log_format(LOG_INFO, "set position %d %d %d", (int)pos.x, (int)pos.y, (int)(pos.theta * 180 / M_PI));
	portENTER_CRITICAL();
	location_pos = pos;
	portEXIT_CRITICAL();
}

void Location::cmdSetPosition(void* arg, void* data)
{
	Location* loc = (Location*) arg;
	VectPlan* pos = (VectPlan*) data;
	loc->setPosition(*pos);
}
