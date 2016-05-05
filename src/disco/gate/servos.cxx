#include "servos.h"
#include "kernel/log.h"

void Servos::setTorque(bool enable)
{
	if (enable == true)
	{
		// Mise sous tension
		parasol.setTorqueLimit(0.85);

		// Angles limites
		parasol.setGoalLimits(0, M_PI_4);
	}else
	{
		// Mise Hors tension
		parasol.setTorqueLimit(0);
	}
}


void Servos::closeAll(void)
{
	// Fermeture de tous les servos (position rangée)
	parasol.setGoalPosition(0);
}

int Servos::setAngle(Dynamixel *servo, float angle, enum ServosWaitPolicy wait)
{
	int result = 0;
	bool stucked = false;
	bool reached = false;

	servo->setGoalPosition(angle);
	log_format(LOG_INFO, "AX12 ID = %d", servo->id());

	if (wait == SERVO_POLICY_WAIT_END)
	{
		do {
			reached = servo->isFlagActive(DYNAMIXEL_FLAG_TARGET_REACHED);
			stucked = servo->isFlagActive(DYNAMIXEL_FLAG_STUCK);
			vTaskDelay(50);
		} while(reached == false && stucked == false);

		if (stucked == true)
		{
			result = -1;
		} else if (reached == true)
		{
			result = 1;
		}
	}
	return result;
}


void Servos::setParasolState(enum Parasol_state parasolState)
{
	switch(parasolState)
	{
		case PARASOL_CLOSE:
			Servos::setAngle(&parasol, 0, SERVO_POLICY_NON_BLOCKING);
			break;
		case PARASOL_OPEN:
			Servos::setAngle(&parasol, 0.5, SERVO_POLICY_WAIT_END);
			break;
		default:
			break;
	}
}
