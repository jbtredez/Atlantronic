//! @file encoders.c
//! @brief Encoders
//! @author Jean-Baptiste Trédez

#include "io/encoders.h"
#include "module.h"
#include "init.h"

static int encoders_module_init()
{
	#warning TODO : initialiser les codeurs et filtres

	return 0;
}

module_init(encoders_module_init, INIT_ENCODERS);

uint16_t encoders_get(unsigned int num)
{
	#warning TODO encodeurs

	return 0;
}

