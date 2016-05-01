#include "PwmMotor.h"
#include "kernel/driver/adc.h"
#include "kernel/control.h"

void PwmMotor::set_speed(float v)
{
	float vBat = adc_filtered_data.vBat;
	float cmd = 0;

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
