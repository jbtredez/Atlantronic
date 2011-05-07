#ifndef CURRENT_H
#define CURRENT_H

//! @file current.h
//! @brief Current
//! @author Atlantronic

#include <stdint.h>

#define CURRENT_MOT_RIGHT    0
#define CURRENT_MOT_LEFT     1

uint32_t current_get(unsigned int num);

#endif
