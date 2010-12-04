//! @file pwm.c
//! @brief PWM
//! @author Jean-Baptiste Trédez

#include "io/pwm.h"
#include "module.h"
#include "init.h"

#define STM32F10X_CL
#include "cpu/stm32f10x.h"

static int pwm_module_init()
{
	#warning TODO : initialiser les pwm

	return 0;
}

module_init(pwm_module_init, INIT_PWM);

void pwm_set(unsigned int num, uint16_t val, int dir)
{
	#warning TODO : pwm_set (temporaire - simu)
	int16_t val2 = (val >> 1) * dir;
	if(num == 1)
	{
		TIM1->CCR1 = val2;
	}
	else
	{
		TIM1->CCR2 = val2;
	}
}
