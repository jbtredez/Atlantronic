#ifndef CONTROL_H
#define CONTROL_H

//! @file control.h
//! @brief Asservissement
//! @author Atlantronic

#include <stdint.h>
#include "kernel/systick.h"
#include "kernel/control/kinematics.h"

//! période de la tache de propulsion en ms ("fréquence" de l'asservissement)
#define CONTROL_PERIOD               5
#define CONTROL_HZ                 200

enum control_state
{
	CONTROL_READY_FREE = 0,       //!< no trajectory ongoing, control off
	CONTROL_READY_ASSER,          //!< no trajectory ongoing, control on
	CONTROL_TRAJECTORY,           //!< trajectoire en cours
	CONTROL_BACK_TO_WALL,         //!< pas d'asservissement, les deux roues en marche arrière, pwm à x %. Arrêt quand le robot ne bouge plus
	CONTROL_END,                  //!< end : halted forever
};

enum control_status
{
	CONTROL_TARGET_REACHED = 0,   //!< cible atteinte
	CONTROL_TARGET_NOT_REACHED,   //!< cible non atteinte
	CONTROL_COLSISION,            //!< collision
	CONTROL_TIMEOUT,              //!< timeout
	CONTROL_IN_MOTION,            //!< trajectorie en cours
};

enum control_speed
{
	CONTROL_SPEED_OK,
	CONTROL_OVER_SPEED,
	CONTROL_UNDER_SPEED,
	CONTROL_WRONG_WAY,
};

enum trajectory_way
{
	TRAJECTORY_ANY_WAY,
	TRAJECTORY_FORWARD,
	TRAJECTORY_BACKWARD
};

enum control_type
{
	CONTROL_LINE_A,     //!< rotation sur place
	CONTROL_LINE_XY,    //!< aller a la position x,y en ligne droite (=> rotation puis avance)
	CONTROL_LINE_XYA,   //!< aller a la position x,y, alpha en ligne droite (=> rotation puis avance puis rotation)
};

#endif
