//! @file vect2.c
//! @brief vect2
//! @author Atlantronic

#include "kernel/math/vect2.h"

//!< @function fx_vect2_scalar_product
//!< @brief calcul le produit scalaire de deux vecteurs u et v
//!< @param vecteur u
//!< @param vecteur v
//!< @return produit scalaire calculé avec les positions en mm
int32_t fx_vect2_scalar_product(const struct fx_vect2* u, const struct fx_vect2* v)
{
	return ((u->x>>16)*(v->x>>16))+((u->y>>16)*(v->y>>16));
}

