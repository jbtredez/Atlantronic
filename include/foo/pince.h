#ifndef PINCE_H
#define PINCE_H

//! @file pince.h
//! @brief Gestion des pinces
//! @author Atlantronic

#include <stdint.h>

enum pince_cmd_type
{
	PINCE_CLOSE,
	PINCE_MIDDLE,
	PINCE_OPEN,
};

void pince_set_position(enum pince_cmd_type left, enum pince_cmd_type right);

struct pince_cmd_arg
{
	uint32_t type_left;
	uint32_t type_right;
};

#endif
