#ifndef KINEMATICS_H
#define KINEMATICS_H

//! @file kinematics.h
//! @brief Asservissement
//! @author Atlantronic

#include <stdint.h>

enum
{
	KINEMATICS_SPEED,
	KINEMATICS_POSITION,
};

struct KinematicsParameters
{
	float vMax;    //!< vitesse maximale
	float aMax;    //!< accélération maximale
	float dMax;    //!< décélération maximale
};

//! cinematique d'un axe ou actionneur (commande ou mesure)
struct Kinematics
{
	float pos;   //!< position
	float v;     //!< vitesse
	float a;     //!< acceleration
	int mode;    //!< KINEMATICS_SPEED ou KINEMATICS_POSITION

	void setSpeed(float wantedSpeed, const KinematicsParameters &param, float dt);
	void setPosition(float wantedPos, float wantedSpeed, KinematicsParameters param, float dt);
	void reset();
};

#endif
