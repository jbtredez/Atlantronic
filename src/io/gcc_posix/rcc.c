//! @file rcc.c
//! @brief Gestion Reset et Clock
//! @author Jean-Baptiste Trédez

#include "io/rcc.h"
#include "module.h"

static int rcc_module_init()
{
	return 0;
}

module_init(rcc_module_init, INIT_RCC);

