#include "ArmTimMotor.h"

ArmTimMotor::ArmTimMotor()
{

}

ArmTimMotor::~ArmTimMotor()
{

}

void ArmTimMotor::update()
{
// FIXME: temporaire. Ce n'est pas correct.
	motor[0].pwm = ((int16_t) MEM.CCR1 ) *2;
	motor[1].pwm = ((int16_t) MEM.CCR2 ) *2;
	motor[2].pwm = ((int16_t) MEM.CCR3 ) *2;
	motor[3].pwm = ((int16_t) MEM.CCR4 ) *2;
	printf("pwm : %i / %i\n", motor[0].pwm, motor[1].pwm);
}
