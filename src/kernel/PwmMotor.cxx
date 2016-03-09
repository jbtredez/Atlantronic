#include "PwmMotor.h"
#include "kernel/driver/pwm.h"

void PwmMotor::set_speed(float v)
{
	pwm_set(pwmId, v * inputGain);
}
