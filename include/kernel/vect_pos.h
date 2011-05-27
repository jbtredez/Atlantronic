#ifndef VECT_POS_H
#define VECT_POS_H

//! @file vect_pos.h
//! @brief vect_pos
//! @author Atlantronic

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

#endif
