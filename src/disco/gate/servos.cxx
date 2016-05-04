#include "servos.h"
#include "kernel/log.h"

void Servos::setTorque(bool enable)
{
	if (enable == true)
	{
		// Mise sous tension
		leftFishWing.setTorqueLimit(1);
		leftFishRemover.setTorqueLimit(1);
		rightFishWing.setTorqueLimit(1);
		rightFishRemover.setTorqueLimit(1);
		leftDoor.setTorqueLimit(1);
		rightDoor.setTorqueLimit(1);
		towerPliers.setTorqueLimit(1);
		towerPliersTidier.setTorqueLimit(1);
		parasol.setTorqueLimit(1);

		// Angles limites
		leftFishWing.setGoalLimits(0, 1.49);
		leftFishRemover.setGoalLimits(-0.5, 0.5);
		rightFishWing.setGoalLimits(-1.49, 0);
		rightFishRemover.setGoalLimits(-0.5, 0.5);
		leftDoor.setGoalLimits(-M_PI_2, M_PI_2);
		rightDoor.setGoalLimits(-M_PI_2, M_PI_2);
		towerPliers.setGoalLimits(0, 0);
		towerPliersTidier.setGoalLimits(0, 0);
		parasol.setGoalLimits(0, M_PI_4);

		// Tol��rence target reached
		rightFishWing.setTargetReachedThreshold(0.1);
	}else
	{
		// Mise Hors tension
		leftFishWing.setTorqueLimit(0);
		leftFishRemover.setTorqueLimit(0);
		rightFishWing.setTorqueLimit(0);
		rightFishRemover.setTorqueLimit(0);
		leftDoor.setTorqueLimit(0);
		rightDoor.setTorqueLimit(0);
		towerPliers.setTorqueLimit(0);
		towerPliersTidier.setTorqueLimit(0);
		parasol.setTorqueLimit(0);
	}
}


void Servos::closeAll(void)
{
	// Fermeture de tous les servos (position rang��e)

	// Fermeture des d��crocheurs, de la pince du parasol et la porte gauche portes
	leftFishRemover.setGoalPosition(0);
	rightFishRemover.setGoalPosition(0);
	towerPliers.setGoalPosition(0);
	parasol.setGoalPosition(0);

	setDoorsState(DOOR_CLOSE);
	// Attente de la fin des mouvements

	// Fermeture des ailes et rangement des ailes
	leftFishWing.setGoalPosition(0);
	rightFishWing.setGoalPosition(0);
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

void Servos::setPumpArmState(enum PumpArm_state)
{
	switch(PumpArm_state)
	{
		case PUMP_ARM_UP:
			Servos::setAngle(&pincerPump, 0, SERVO_POLICY_WAIT_END);
			Servos::setAngle(&armPump, 0, SERVO_POLICY_NON_BLOCKING);
			break;
		case PUMP_ARM_DOWN:
			Servos::setAngle(&pincerPump, 0, SERVO_POLICY_WAIT_END);
			Servos::setAngle(&armPump, 0.5, SERVO_POLICY_WAIT_END);
			break;
		case PUMP_ARM_OPEN:
			Servos::setAngle(&armPump, 0.5, SERVO_POLICY_WAIT_END);
			Servos::setAngle(&pincerPump, 0.25, SERVO_POLICY_WAIT_END);
			break;
		default:
			break;
	}
}
