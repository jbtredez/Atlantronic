#ifndef WING_H
#define WING_H

//! @file pince.h
//! @brief Gestion des ailes
//! @author Atlantronic

#include <stdint.h>

enum wing_cmd_type
{
	WING_PARK,
	WING_OPEN,
};

void wing_set_position(enum wing_cmd_type left, enum wing_cmd_type right);

struct wing_cmd_arg
{
	uint32_t type_left;
	uint32_t type_right;
} __attribute__((packed));

#endif
