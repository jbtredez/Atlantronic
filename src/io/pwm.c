//! @file pwm.c
//! @brief PWM
//! @author Jean-Baptiste Trédez

#include "io/pwm.h"
#include "module.h"
#include "init.h"

static int pwm_module_init()
{
	#ifdef __GCC_PIC32__
	#warning TODO : initialiser les pwm
	#endif

	return 0;
}

module_init(pwm_module_init, INIT_PWM);

void pwm_set(int num, uint32_t val)
{
	#warning TODO : pwm_set
}
