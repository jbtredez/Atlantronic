//! @file pwm.c
//! @brief PWM
//! @author Jean-Baptiste Trédez

#include "io/pwm.h"
#include "module.h"
#include "init.h"
#include "simu/model.h"

void pwm_set(unsigned int num, uint16_t val, int dir)
{
	model_pwm_set(num, val, dir);
}
