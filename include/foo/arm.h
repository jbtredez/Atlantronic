#ifndef ARM_H
#define ARM_H

//! @file arm.h
//! @brief Gestion du bras
//! @author Atlantronic

#include <stdint.h>

//!< met le bras à la position souhaitée (coordonées articulaires)
//!< @param z : hauteur (en 2^-16 mm)
//!< @param a : angle du permier ax12 (2^-26 tours)
//!< @param b : angle du second ax12 (2^-26 tours)
//!< @return 0 si c'est possible, -1 sinon
int arm_goto_zab(uint32_t z, int32_t a, int32_t b);

//!< met le bras au dessus de la position (x, y, z) (repère robot) en fonction
//!< du plus court chemin
int arm_goto_xyz(int32_t x, int32_t y, uint32_t z);

struct arm_cmd_zab_param
{
	uint32_t z;
	int32_t a;
	int32_t b;
} __attribute__((packed));

#endif