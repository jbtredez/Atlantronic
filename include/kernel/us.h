#ifndef US_H
#define US_H

//! @file us.h
//! @brief CAN - US
//! @author Atlantronic

#include <stdint.h>

//! TODO : trouver un nom selon la position
enum us_id
{
	US_1,
	US_2,
	US_3,
	US_4,
	US_5,
	US_MAX
};

uint32_t us_get_state(enum us_id id);

#endif
