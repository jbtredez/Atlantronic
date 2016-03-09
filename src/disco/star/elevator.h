#ifndef ELEVATOR_H
#define ELEVATOR_H

//! @file elevator.h
//! @brief Gestion de l'ascenseur
//! @author Atlantronic

#ifndef WEAK_ELEVATOR
#define WEAK_ELEVATOR __attribute__((weak, alias("nop_function") ))
#endif

#include <stdint.h>
#include "kernel/driver/io.h"

void elevator_set_position(float pos);

float elevator_get_position() WEAK_ELEVATOR;

inline bool elevator_omron_active()
{
	return gpio_get(IO_OMRON_ELEVATOR) != 0;
}

struct elevator_cmd_arg
{
		float pos;
}__attribute__((packed));

#endif
