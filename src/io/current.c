//! @file current_sensors.c
//! @brief Current sensors
//! @author Jean-Baptiste Trédez

#include "io/current.h"
#include "module.h"
#include "init.h"

#ifdef __GCC_POSIX__
#include "simu/model.h"
#endif

#ifdef __GCC_PIC32__
static int current_module_init()
{
	#warning TODO : initialiser

	return 0;
}

module_init(current_module_init, INIT_CURRENT);

#endif

uint32_t current_get(unsigned int num)
{
	#ifdef __GCC_POSIX__
	return model_current_get(num);
	#endif

	#ifdef __GCC_PIC32__
	#warning TODO current
	#endif

	return 0;
}

