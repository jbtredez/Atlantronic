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
		parasol.setGoalLimits(-M_PI_2, 0);

		// Tolérence target reached
		leftFishWing.setTargetReachedThreshold(0.1);
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
	// Fermeture de tous les servos (position rangée)

	// Fermeture des décrocheurs, de la pince du parasol et la porte gauche portes
	leftFishRemover.setGoalPosition(0);
	rightFishRemover.setGoalPosition(0);
	towerPliers.setGoalPosition(0);
	parasol.setGoalPosition(-0.05);

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

void Servos::setWingState(enum Wing_state left, enum Wing_state right)
{
	rightFishWing.setMovingSpeed(0.85);
	leftFishWing.setMovingSpeed(0.85);
	switch(right)
	{
		case WING_OPEN:
			setAngle(&rightFishWing, -1.5, SERVO_POLICY_WAIT_END);
			break;
		case WING_MIDDLE:
			setAngle(&rightFishWing, -1.13, SERVO_POLICY_WAIT_END);
			break;
		case WING_CLOSE:
			setAngle(&rightFishWing, 0, SERVO_POLICY_WAIT_END);
			break;
		case WING_NO_MOVE:
			break;
		default:
			break;
	}

	switch(left)
	{
		case WING_OPEN:
			setAngle(&leftFishWing, 1.5, SERVO_POLICY_WAIT_END);
			break;
		case WING_MIDDLE:
			setAngle(&leftFishWing, 1.13, SERVO_POLICY_WAIT_END);
			break;
		case WING_CLOSE:
			setAngle(&leftFishWing, 0, SERVO_POLICY_WAIT_END);
			break;
		case WING_NO_MOVE:
			break;
		default:
			break;
	}
}


void Servos::setFishRemoverState(enum FishRemover_state left, enum FishRemover_state right)
{
	switch(right)
	{
		case FISH_REMOVER_TIDY:
			Servos::setAngle(&rightFishRemover, 0, SERVO_POLICY_NON_BLOCKING);
			break;
		case FISH_REMOVER_SHAKE:
			Servos::setAngle(&rightFishRemover, 0.5, SERVO_POLICY_WAIT_END);
			Servos::setAngle(&rightFishRemover, -0.5, SERVO_POLICY_WAIT_END);
			Servos::setAngle(&rightFishRemover, 0, SERVO_POLICY_NON_BLOCKING);
			break;
		case FISH_REMOVER_NO_MOVE:
			break;
		default:
			break;
	}

	switch(left)
	{
		case FISH_REMOVER_TIDY:
			Servos::setAngle(&leftFishRemover, 0, SERVO_POLICY_NON_BLOCKING);
			break;
		case FISH_REMOVER_SHAKE:
			Servos::setAngle(&leftFishRemover, -0.5, SERVO_POLICY_WAIT_END);
			Servos::setAngle(&leftFishRemover, 0.5, SERVO_POLICY_WAIT_END);
			Servos::setAngle(&leftFishRemover, 0, SERVO_POLICY_NON_BLOCKING);
			break;
		case FISH_REMOVER_NO_MOVE:
			break;
		default:
			break;
	}
}

void Servos::setDoorsState(enum Door_state doorState)
{
	switch(doorState)
	{
		case DOOR_CLOSE:
			Servos::setAngle(&leftDoor, M_PI_2, SERVO_POLICY_WAIT_END);
			Servos::setAngle(&rightDoor,-M_PI_2, SERVO_POLICY_WAIT_END);
			break;
		case DOOR_OPEN:
			Servos::setAngle(&leftDoor, 0, SERVO_POLICY_WAIT_END);
			Servos::setAngle(&rightDoor, 0, SERVO_POLICY_WAIT_END);
			break;
		case DOOR_OPEN_WIDE:
			Servos::setAngle(&leftDoor, -M_PI_2, SERVO_POLICY_WAIT_END);
			Servos::setAngle(&rightDoor, M_PI_2, SERVO_POLICY_WAIT_END);
			break;
		case DOOR_GRIP:
			Servos::setAngle(&leftDoor, 0.2, SERVO_POLICY_NON_BLOCKING);
			Servos::setAngle(&rightDoor, -0.2, SERVO_POLICY_NON_BLOCKING);
			break;
		default:
			break;
	}
}


void Servos::setParasolState(enum Parasol_state parasolState)
{
	switch(parasolState)
	{
		case PARASOL_CLOSE:
			Servos::setAngle(&parasol, -0.05, SERVO_POLICY_NON_BLOCKING);
			break;
		case PARASOL_OPEN:
			Servos::setAngle(&parasol, (float)(-70*M_PI/180), SERVO_POLICY_WAIT_END); // -70 °
			break;
		default:
			break;
	}
}

void Servos::setTowerPlierState(enum TowerPlier_state plierState)
{
	switch(plierState)
	{
		case TOWER_PLIER_TIDY:
			Servos::setAngle(&towerPliers, 0, SERVO_POLICY_WAIT_END);
			Servos::setAngle(&towerPliersTidier, 0, SERVO_POLICY_NON_BLOCKING);
			break;
		case TOWER_PLIER_CLOSE:
			Servos::setAngle(&towerPliersTidier, M_PI_2, SERVO_POLICY_WAIT_END);
			Servos::setAngle(&towerPliers, -0.5, SERVO_POLICY_WAIT_END);
			break;
		case TOWER_PLIER_OPEN:
			Servos::setAngle(&towerPliersTidier, M_PI_2, SERVO_POLICY_WAIT_END);
			Servos::setAngle(&towerPliers, 0.5, SERVO_POLICY_WAIT_END);
			break;
		case TOWER_PLIER_GRIP:
			Servos::setAngle(&towerPliersTidier, M_PI_2, SERVO_POLICY_WAIT_END);
			Servos::setAngle(&towerPliers, -0.1, SERVO_POLICY_WAIT_END);
			break;
		default:
			break;
	}
}
