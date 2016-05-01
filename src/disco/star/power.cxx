
#include "disco/power.h"
#include "disco/gate/gate.h"
#include "kernel/PwmMotor.h"

extern PwmMotor motionMotors[MOTION_MOTOR_MAX];

void cutMotorsOff(void)
{

}
void cutServosOff(void)
{
	funnyAction();
	Servos::setTorque(false);

}
void funnyAction(void)
{
	Servos::setParasolState(PARASOL_OPEN);
}
