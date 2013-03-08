#ifndef DISTANCE_H
#define DISTANCE_H

//! @file distance.h
//! @brief Fonctions de calcul de distances
//! @author Atlantronic

#include <stdint.h>
#include "kernel/math/vect2.h"

//!< @function distance_point_to_point_squared
//!< @brief calcule le carré de la distance entre deux points A et B
//!< @param a coordonnées du point A
//!< @param b coordonnées du point B
//!< @return distance en mm^2
int32_t distance_point_to_point_squared(const struct fx_vect2* a, const struct fx_vect2* b);

//!< @function distance_point_to_point
//!< @brief calcule la distance entre deux points A et B
//!< @param a coordonnées du point A
//!< @param b coordonnées du point B
//!< @return distance en mm << 16
int32_t distance_point_to_point(const struct fx_vect2* a, const struct fx_vect2* b);

//!< @function distance_point_to_segment
//!< @brief calcule la distance entre un point M et un segment [AB]
//!< @param m coordonnées du point M
//!< @param a coordonnées du point A
//!< @param b coordonnées du point B
//!< @return distance en 2^-16 mm
int32_t distance_point_to_segment(const struct fx_vect2* m, const struct fx_vect2* a, const struct fx_vect2* b);

#endif
