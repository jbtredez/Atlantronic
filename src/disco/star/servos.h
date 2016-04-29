#ifndef SRC_DISCO_STAR_SERVOS_H_
#define SRC_DISCO_STAR_SERVOS_H_

#include "star.h"

enum Wing_state
{
	WING_OPEN,		// Ouvert 90 degrés
	WING_CLOSE,		// Fermé sur le flanc du robot
	WING_MIDDLE,	// Ouvert 45 degrés
	WING_NO_MOVE,	// Pas bouger
};

enum FishRemover_state
{
	FISH_REMOVER_TIDY,		// Position milieux collé à l'aile
	FISH_REMOVER_FORWARD,	// Rotation vers l'avant du robot
	FISH_REMOVER_BACKWARD,	// Rotation vers l'arrière du robot
};

enum Door_state
{
	DOOR_CLOSE,		// Fermé dans le robot
	DOOR_OPEN,		// Ouvert à 90 degrés de la face avant
	DOOR_OPEN_WIDE,	// Ouvert large à 180 degrés de la face avant
	DOOR_GRIP,		// Position de serrage
};

enum Parasol_state
{
	PARASOL_CLOSE,
	PARASOL_OPEN,
};

enum TowerPlier_state
{
	TOWER_PLIER_TIDY,		// Position rangée
	TOWER_PLIER_CLOSE,		// Fermé dans le robot
	TOWER_PLIER_OPEN,		// Ouvert
	TOWER_PLIER_GRIP,		// Position de serrage
};

enum ServosWaitPolicy
{
	SERVO_POLICY_NON_BLOCKING,
	SERVO_POLICY_WAIT_END,
};


class Servos
{
	public:
	static void setTorque(bool enable);
	static void closeAll(void);
	static int setAngle(Dynamixel *servo, float angle, enum ServosWaitPolicy);
	static void setWingState(enum Wing_state left, enum Wing_state right);
};

#endif /* SRC_DISCO_STAR_SERVOS_H_ */
