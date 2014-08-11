#ifndef ENCODER_H
#define ENCODER_H

//! @file encoder.h
//! @brief Encoder
//! @author Atlantronic

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

enum
{
	ENCODER_1 = 0,
	ENCODER_2,
	ENCODER_3,
	ENCODER_MAX
};

uint16_t encoder_get(const unsigned int id);

#ifdef __cplusplus
}
#endif

#endif
