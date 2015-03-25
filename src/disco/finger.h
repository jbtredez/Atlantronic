#ifndef FINGER_H
#define FINGER_H

//! @file finger.h
//! @brief Gestion des doigt de l'ascenseur
//! @author Atlantronic

#include <stdint.h>

enum finger_type
{
	FINGER_CLOSE,
	FINGER_HALF_CLOSE,
	FINGER_OPEN,
};

void finger_set_pos(enum finger_type low, enum finger_type high);

struct finger_cmd_arg
{
	uint32_t low;
	uint32_t high;
}__attribute__((packed));

#endif
