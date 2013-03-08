#ifndef VECT_POS_H
#define VECT_POS_H

//! @file vect_pos.h
//! @brief vect_pos
//! @author Atlantronic

#include <stdint.h>
#include "kernel/math/vect2.h"

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

void vect2_loc_to_abs(const struct fx_vect_pos *origin, const struct fx_vect2 *pos_in, struct fx_vect2 *pos_out);

void vect2_abs_to_loc(const struct fx_vect_pos *origin, const struct fx_vect2 *pos_in, struct fx_vect2 *pos_out);

void pos_loc_to_abs(const struct fx_vect_pos *origin, const struct fx_vect_pos *pos_in, struct fx_vect_pos *pos_out);

void pos_abs_to_loc(const struct fx_vect_pos *origin, const struct fx_vect_pos *pos_in, struct fx_vect_pos *pos_out);

#endif