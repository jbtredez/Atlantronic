#ifndef VECT_POS_H
#define VECT_POS_H

//! @file vect_pos.h
//! @brief vect_pos
//! @author Atlantronic

#include <stdint.h>

struct vect2i
{
	int x;
	int y;
};

struct fx_vect2
{
	int32_t x;
	int32_t y;
};

struct fx_vect_pos
{
	int32_t x;
	int32_t y;
	int32_t alpha;
	int32_t ca;
	int32_t sa;
};

//! @struct vect_pos
//! représentation d'un vecteur sur la table
struct vect_pos
{
	float x;          //!< coordonnée selon l'axe x
	float y;          //!< coordonnée selon l'axe y
	float alpha;      //!< orientation
	float ca;         //!< cos(alpha)
	float sa;         //!< sin(alpha)
};

void pos_robot_to_table(struct vect_pos *pos_robot, struct vect_pos *pos_in, struct vect_pos *pos_out);

void pos_table_to_robot(struct vect_pos *pos_robot, struct vect_pos *pos_in, struct vect_pos *pos_out);

float norm2_square(struct vect_pos *pos);

float distance_square(struct vect_pos *pos1, struct vect_pos *pos2);

#endif
