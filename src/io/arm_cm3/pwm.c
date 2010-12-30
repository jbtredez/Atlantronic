//! @file pwm.c
//! @brief PWM
//! @author Jean-Baptiste Trédez

#include "io/pwm.h"
#include "module.h"
#include "init.h"
#include "cpu/cpu.h"

static int pwm_module_init()
{
	#warning TODO : initialiser les pwm (temporaire - simu)
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->RESERVED0 = 1;

	return 0;
}

module_init(pwm_module_init, INIT_PWM);

void pwm_set(unsigned int num, uint16_t val, int dir)
{
	#warning TODO : pwm_set (temporaire - simu)
	int16_t val2 = (val >> 1) * dir;
	if(num == PWM_RIGHT)
	{
		TIM1->CCR1 = val2;
	}
	else
	{
		TIM1->CCR2 = val2;
	}
}
