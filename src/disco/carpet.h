#ifndef CARPET_H
#define CARPET_H

//! @file carpet.h
//! @brief Gestion des tapis
//! @author Atlantronic

#include <stdint.h>

enum carpet_type
{
	CARPET_UP,
	CARPET_DOWN,
};

void carpet_set_pos(enum carpet_type right, enum carpet_type left);

struct carpet_cmd_arg
{
	uint32_t right;
	uint32_t left;
}__attribute__((packed));

#endif
