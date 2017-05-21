#include "servos.h"
#include "kernel/log.h"

void Servos::setTorque(bool enable)
{
	if (enable == true)
	{
		// Set torque value
		missileLeft.setTorqueLimit(0.85);
		missileRight.setTorqueLimit(0.85);
		pusher.setTorqueLimit(0.85);

		// Set AX12 limit angles
		missileLeft.setGoalLimits( -M_PI_2, -0.2 );
		missileRight.setGoalLimits( 0.2, M_PI_2 );
		pusher.setGoalLimits( -M_PI_2, 0 );

		pusher.setTorqueEnable(1);
		//missileLeft.setTorqueEnable(1);	// Pas pour les missiles: asservissement seulement avant la funny action
		//missileRight.setTorqueEnable(1);	// Pas pour les missiles: asservissement seulement avant la funny action


		// Set default angles
		missileLeft.setGoalPosition(-0.3);
		//missileRight.setGoalPosition(0.3);

	}else
	{
		// Mise Hors tension
		//missileLeft.setTorqueLimit(0); 	// Fin de match: pas de hors tension
		//missileRight.setTorqueLimit(0);
		pusher.setTorqueLimit(0);
	}
}


void Servos::closeAll(void)
{
	// Fermeture de tous les servos (position rangée)

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
