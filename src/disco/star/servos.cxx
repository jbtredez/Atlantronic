#include "servos.h"
#include "kernel/log.h"

void Servos::setTorque(bool enable)
{
	if (enable == true)
	{
		// Mise sous tension
		NetTrap.setTorqueLimit(1);


		// Angles limites
		NetTrap.setGoalLimits(0, 1.49);


		// Tolérence target reached
		NetTrap.setTargetReachedThreshold(0.1);

	}else
	{
		// Mise Hors tension
		NetTrap.setTorqueLimit(0);

	}
}


void Servos::closeAll(void)
{
	// Fermeture de tous les servos (position rangée)

	// Fermeture des décrocheurs, de la pince du parasol et la porte gauche portes
	NetTrap.setGoalPosition(0);

	setNetState(NET_TRAP_CLOSE);



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



void Servos::setNetState(enum Net_Trap_state NetState)
{
	switch(NetState)
	{
		case NET_TRAP_CLOSE:
			Servos::setAngle(&NetTrap, M_PI_2, SERVO_POLICY_WAIT_END);
			break;
		case NET_TRAP_OPEN:
			Servos::setAngle(&NetTrap, 0, SERVO_POLICY_WAIT_END);
			break;
		default:
			break;
	}
}


