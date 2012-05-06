#ifndef ARM_H
#define ARM_H

//! @file arm.h
//! @brief Gestion du bras
//! @author Atlantronic

#include <stdint.h>

enum arm_cmd_type
{
	ARM_CMD_ART,
	ARM_CMD_XYZ_LOC,
	ARM_CMD_XYZ_ABS
};

//!< met le bras à la position souhaitée (coordonées articulaires)
//!< @param z : hauteur (en 2^-16 mm)
//!< @param a : angle du permier ax12 (2^-26 tours)
//!< @param b : angle du second ax12 (2^-26 tours)
//!< @return 0 si c'est possible, -1 sinon
int arm_goto_abz(uint32_t z, int32_t a, int32_t b);

//!< met le bras au dessus de la position (x, y, z) (repère robot ou absolu selon le type)
//!< en fonction du plus court chemin
int arm_goto_xyz(int32_t x, int32_t y, uint32_t z, enum arm_cmd_type type);

//!< mise en marche de la pompe
void arm_bridge_on();

//!< arrêt de la pompe
void arm_bridge_off();

struct arm_cmd_goto_param
{
	union{
		struct
		{
			int32_t x;
			int32_t y;
		};
		struct
		{
			int32_t a;
			int32_t b;
		};
	};
	uint32_t z;
	uint8_t type;
} __attribute__((packed));

#endif