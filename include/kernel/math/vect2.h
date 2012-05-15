#ifndef VECT2_H
#define VECT2_H

//! @file vect2.h
//! @brief vect2
//! @author Atlantronic

#include <stdint.h>
#include "kernel/math/fx.h"

struct fx_vect2
{
	int32_t x;
	int32_t y;
};

//!< @function fx_vect2_scalar_product
////!< @brief calcul le produit scalaire de deux vecteurs u et v
////!< @param vecteur u
////!< @param vecteur v
////!< @return produit scalaire calculé avec les positions en mm
int32_t fx_vect2_scalar_product(const struct fx_vect2* u, const struct fx_vect2* v);

#endif
