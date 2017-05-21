#include "PwmMotor.h"
#include "kernel/driver/adc.h"
#include "kernel/control.h"

void PwmMotor::set_speed(float v)
{
	float vBat = adc_filtered_data.vBat;
	float cmd = 0;

/*	if (fabs(v) > 0 && pwmId == 0)
	{
		log_format(LOG_INFO, "V = %d", (int)v);
	}
*/
	encoder->update(CONTROL_DT);
	float error = v - encoder->getSpeed();
	v += pid.compute(error, CONTROL_DT);
	cmd = v * inputGain / vBat;

	if( vBat < ADC_VBAT_UNDERVOLTAGE )
	{
		cmd = 0;
		pid.reset();
	}
	pwm_set(pwmId, cmd);
}

bool PwmMotor::is_in_motion()
{
	float encoderSpeed = encoder->getSpeed();
	return encoderSpeed > 1;
}


void PwmMotor::disable()
{
	pwm_disable();
}

void PwmMotor::stop_on_collision()
{
	pid.reset();
}

