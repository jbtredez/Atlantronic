#define WEAK_FUNNYACTION
#define WEAK_CUTSERVOSOFF
#define WEAK_CUTMOTORSOFF
#include "disco/power.h"
#include "disco/gate/gate.h"
#include "kernel/PwmMotor.h"
#include "disco/gate/servos.h"

extern PwmMotor motionMotors[MOTION_MOTOR_MAX];

void cutMotorsOff(void)
{
	motionMotors[MOTION_MOTOR_LEFT].disable();
	motionMotors[MOTION_MOTOR_RIGHT].disable();
}
void cutServosOff(void)
{
	funnyAction();
	//Servos::setTorque(false);

}
void funnyAction(void)
{

}
