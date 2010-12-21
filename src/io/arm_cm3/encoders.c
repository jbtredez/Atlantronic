//! @file encoders.c
//! @brief Encoders
//! @author Jean-Baptiste Trédez

#include "io/encoders.h"
#include "module.h"
#include "init.h"

#define STM32F10X_CL
#include "cpu/stm32f10x.h"

static int encoders_module_init()
{
	#warning TODO : initialiser les codeurs et filtres (temporaire - simu)
	TIM3->CCR1 = 0;
	TIM4->CCR1 = 0;

	return 0;
}

module_init(encoders_module_init, INIT_ENCODERS);

uint16_t encoders_get(unsigned int num)
{
	#warning TODO : encoders_get (temporaire - simu)
	uint16_t val;
	if(num == ENCODERS_MOT_RIGHT)
	{
		val = TIM3->CCR1;
	}
	else
	{
		val = TIM4->CCR1;
	}

	return val;
}

