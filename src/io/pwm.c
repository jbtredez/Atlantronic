//! @file pwm.c
//! @brief PWM
//! @author Jean-Baptiste Trédez

#include "io/pwm.h"
#include "module.h"
#include "init.h"

#ifdef __GCC_POSIX__
#include "simu/model.h"
#endif

#ifdef __GCC_PIC32__
static int pwm_module_init()
{

	#warning TODO : initialiser les pwm

	return 0;
}

module_init(pwm_module_init, INIT_PWM);

#endif

void pwm_set(unsigned int num, uint32_t val, int dir)
{
	#ifdef __GCC_PIC32__
	#warning TODO : pwm_set
	#endif

	#ifdef __GCC_POSIX__
	model_pwm_set(num, val, dir);
	#endif
}
