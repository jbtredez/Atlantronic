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
	FINGER_GOBLET,
	FINGER_HALF_OPEN,
	FINGER_OPEN,
};

enum finger_bottom_type
{
	FINGER_BOTTOM_CLOSE,
	FINGER_BOTTOM_OPEN,
};

void finger_set_pos(enum finger_type low, enum finger_type high);

void finger_bottom_set_pos(enum finger_bottom_type right, enum finger_bottom_type left);

struct finger_cmd_arg
{
	uint32_t low;
	uint32_t high;
	uint32_t right;
	uint32_t left;
}__attribute__((packed));

#endif
