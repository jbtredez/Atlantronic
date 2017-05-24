#ifndef ESC_H
#define ESC_H

//! @file esp.h
//! @brief ESP
//! @author Atlantronic

#include <stdint.h>


void escs_enable(bool enabled);

class Esc
{
	public:
		void setVal(float cmd);

};
#endif
