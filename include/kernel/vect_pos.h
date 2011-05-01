#ifndef VECT_POS_H
#define VECT_POS_H

//! @file vect_pos.h
//! @brief vect_pos
//! @author Jean-Baptiste Trédez

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

#endif
