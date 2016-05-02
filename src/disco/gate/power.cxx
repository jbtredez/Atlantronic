
#include "disco/power.h"
#include "disco/gate/gate.h"
#include "kernel/PwmMotor.h"

extern PwmMotor motionMotors[MOTION_MOTOR_MAX];

void cutMotorsOff(void)
{
	motionMotors[MOTION_MOTOR_LEFT].disable();
	motionMotors[MOTION_MOTOR_RIGHT].disable();
}
void cutServosOff(void)
{
	funnyAction();

}
void funnyAction(void)
{

}
