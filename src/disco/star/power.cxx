#define WEAK_FUNNYACTION
#define WEAK_CUTSERVOSOFF
#define WEAK_CUTMOTORSOFF
#include "disco/power.h"
#include "disco/star/star.h"
#include "kernel/PwmMotor.h"
#include "disco/star/servos.h"

extern PwmMotor motionMotors[MOTION_MOTOR_MAX];

void cutMotorsOff(void)
{

}
void cutServosOff(void)
{
	Servos::closeAll();
	funnyAction();
	Servos::setTorque(false);

}
void funnyAction(void)
{

}
