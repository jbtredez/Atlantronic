//! @file encoders.c
//! @brief Encoders
//! @author Jean-Baptiste Trédez

#include "io/encoders.h"
#include "module.h"
#include "init.h"

#ifdef __GCC_POSIX__
#include "simu/model.h"
#endif

#ifdef __GCC_PIC32__
static int encoders_module_init()
{
	#warning TODO : initialiser les codeurs et filtres

	return 0;
}

module_init(encoders_module_init, INIT_ENCODERS);

#endif

uint16_t encoders_get(unsigned int num)
{
	#ifdef __GCC_POSIX__
	return model_encoders_get(num);
	#endif

	#ifdef __GCC_PIC32__
	#warning TODO encodeurs
	#endif

	return 0;
}

