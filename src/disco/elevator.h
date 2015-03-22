#ifndef ELEVATOR_H
#define ELEVATOR_H

//! @file elevator.h
//! @brief Gestion de l'ascenseur
//! @author Atlantronic

#include <stdint.h>

void elevator_set_position(float pos);

struct elevator_cmd_arg
{
		float pos;
}__attribute__((packed));

#endif
