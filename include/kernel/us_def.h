#ifndef US_DEF_H
#define US_DEF_H

//! @file us_def.h
//! @brief us_def.h
//! @author Atlantronic

#include <stdint.h>
	
#define US_RIGHT_MASK         0x01
#define US_FRONT_MASK         0x02
#define US_BACK_MASK          0x04
#define US_LEFT_MASK          0x08
#define US_NA_MASK            0x10

enum us_id
{
	US_RIGHT = 0,   // US0
	US_FRONT,       // US1
	US_BACK,        // US2
	US_LEFT,        // US4
	US_NA,          // US3
	US_MAX
};

#endif
