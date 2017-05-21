#ifndef ESC_H
#define ESC_H

//! @file esp.h
//! @brief ESP
//! @author Atlantronic

#include <stdint.h>
static void cmd_esc(void* arg, void* data);

class Esc
{
	public:
		void SetVal(float cmd);
};
#endif
