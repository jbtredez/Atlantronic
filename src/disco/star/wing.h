#ifndef WING_H
#define WING_H

//! @file wing.h
//! @brief Gestion des ailes
//! @author Atlantronic

#include <stdint.h>

enum wing_cmd_type
{
	WING_PARK,
	WING_OPEN,
};
////////////////////////////////////////////////
/// function    : wing_set_position()
/// descrition  : Open Wings
/// param       : left = enum wing_cmd_type left wing state
/// param       : right = enum wing_cmd_type right wing state
/// retrun      : none
////////////////////////////////////////////////
void wing_set_position(enum wing_cmd_type left, enum wing_cmd_type right);

////////////////////////////////////////////////
/// function    : wing_set_position()
/// descrition  : Open Wings
/// param       : sym = bool use symetric commande
/// param       : left = enum wing_cmd_type left wing state
/// param       : right = enum wing_cmd_type right wing state
/// retrun      : none
////////////////////////////////////////////////
void wing_set_position(bool sym, enum wing_cmd_type left, enum wing_cmd_type right);

struct wing_cmd_arg
{
	uint32_t type_left;
	uint32_t type_right;
} __attribute__((packed));

#endif
