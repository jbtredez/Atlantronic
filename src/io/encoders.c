//! @file encoders.c
//! @brief Encoders
//! @author Jean-Baptiste Trédez

#include "io/encoders.h"
#include "module.h"
#include "init.h"

static int encoders_module_init()
{
	#ifdef __GCC_PIC32__
	#warning TODO : initialiser les codeurs et filtres
	#endif

	return 0;
}

module_init(encoders_module_init, INIT_ENCODERS);

uint16_t encoders_get_right()
{
	#warning TODO encodeurs

	return 0;
}

uint16_t encoders_get_left()
{
	#warning TODO encodeurs

	return 0;
}
