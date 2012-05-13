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
	ARM_CMD_XYZ_ABS,
	ARM_CMD_VENTOUSE_ABS,
	ARM_CMD_HOOK_ABS,
};

//!< met le bras à la position souhaitée (coordonées articulaires)
//!< @param a : angle du permier ax12 (2^-26 tours)
//!< @param b : angle du second ax12 (2^-26 tours)
//!< @param z : hauteur (en 2^-16 mm)
//!< @return 0 si c'est possible, -1 sinon
int arm_goto_abz(int32_t a, int32_t b, uint32_t z);

//!< met le bras au dessus de la position (x, y, z) (repère robot ou absolu selon le type)
//!< en fonction du plus court chemin
//!< @param x : x en 2^16 mm
//!< @param y : y en 2^16 mm
//!< @param z : hauteur (en 2^-16 mm)
int arm_goto_xyz(int32_t x, int32_t y, uint32_t z, enum arm_cmd_type type);

//!< place la ventouse perpendiculairement au segment [(x1, y1, z) (x2, y2, z)]
int arm_ventouse_goto(int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint32_t z, int8_t tool_way);

//!< place le crochet perpendiculairement au segment [(x1, y1, z) (x2, y2, z)]
int arm_hook_goto(int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint32_t z, int8_t tool_way);

//!< orientation de l'outil
int arm_set_tool_way(int8_t tool_way);

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
			int32_t x1;
			int32_t y1;
		};
		struct
		{
			int32_t a;
			int32_t b;
		};
	};
	uint32_t z;
	int32_t x2;
	int32_t y2;
	int8_t tool_way;
	uint8_t type;
} __attribute__((packed));

#endif