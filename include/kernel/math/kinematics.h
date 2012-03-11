#ifndef KINEMATICS_H
#define KINEMATICS_H

//! @file kinematics.h
//! @brief Cinematique du robot
//! @author Atlantronic

#include <stdint.h>

struct kinematics
{
	int32_t x;          //!< coordonnée selon l'axe x en 2^-16 mm
	int32_t y;          //!< coordonnée selon l'axe y en 2^-16 mm
	int32_t alpha;      //!< orientation en 2^-26 tours
	int32_t ca;         //!< fx_cos(alpha)
	int32_t sa;         //!< fx_sin(alpha)
	int32_t v;          //!< vitesse d'avance en 2^-16 mm / unite
	int32_t w;          //!< vitesse de rotation en 2^-26 tours / unite
};

#endif