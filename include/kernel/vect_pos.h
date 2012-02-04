#ifndef VECT_POS_H
#define VECT_POS_H

//! @file vect_pos.h
//! @brief vect_pos
//! @author Atlantronic

#include <stdint.h>

struct fx16_vect2
{
	int16_t x;
	int16_t y;
};

struct fx_vect2
{
	int32_t x;
	int32_t y;
};

//! @struct fx_vect_pos
//! représentation d'un vecteur sur la table
struct fx_vect_pos
{
	int32_t x;          //!< coordonnée selon l'axe x en 2^-16 mm
	int32_t y;          //!< coordonnée selon l'axe y en 2^-16 mm
	int32_t alpha;      //!< orientation en 2^-24 tours
	int32_t ca;
	int32_t sa;
};

void fx_vect2_robot_to_table(struct fx_vect_pos *pos_robot, struct fx_vect2 *pos_in, struct fx_vect2 *pos_out);

void fx_vect2_table_to_robot(struct fx_vect_pos *pos_robot, struct fx_vect2 *pos_in, struct fx_vect2 *pos_out);

void pos_robot_to_table(struct fx_vect_pos *pos_robot, struct fx_vect_pos *pos_in, struct fx_vect_pos *pos_out);

void pos_table_to_robot(struct fx_vect_pos *pos_robot, struct fx_vect_pos *pos_in, struct fx_vect_pos *pos_out);

#endif
