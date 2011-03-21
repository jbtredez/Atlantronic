#ifndef ENCODERS_H
#define ENCODERS_H

//! @file encoders.h
//! @brief Encoders
//! @author Jean-Baptiste Trédez

#include <stdint.h>

#define ENCODERS_MOT_RIGHT    0
#define ENCODERS_MOT_LEFT     1

uint16_t encoders_get(const unsigned int num);

#endif
