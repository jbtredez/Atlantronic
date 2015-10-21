#ifndef LOCATION_H
#define LOCATION_H

//! @file location.h
//! @brief Location
//! @author Atlantronic

#include <stdint.h>
#include "kernel/math/VectPlan.h"
#include "kernel/control/kinematics.h"

#ifndef LINUX

#ifndef WEAK_LOCATION
#define WEAK_LOCATION __attribute__((weak, alias("nop_function") ))
#endif

class Location
{
	public:
		void init();

		void update(double voie_inv, Kinematics* kinematics_mes, float dt);

		VectPlan getPosition() WEAK_LOCATION;

		VectPlan getSpeed();

		void setPosition(VectPlan pos);

	protected:
		static void cmdSetPosition(void* arg, void* data);
		VectPlan location_pos;
		VectPlan location_speed;
};
#endif

#endif
