#ifndef ENCODER_H
#define ENCODER_H

//! @file encoder.h
//! @brief Encoder
//! @author Atlantronic

#include <stdint.h>

enum
{
	ENCODER_1 = 0,
	ENCODER_2,
	ENCODER_3,
	ENCODER_MAX
};

uint16_t encoder_get(const unsigned int num);

#endif
