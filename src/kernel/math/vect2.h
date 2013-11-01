#ifndef VECT2_H
#define VECT2_H

//! @file vect2.h
//! @brief vect2
//! @author Atlantronic

#include <stdint.h>

struct fx_vect2
{
	int32_t x;
	int32_t y;
};

//!< @function fx_vect2_sum
//!< @brief calcule la somme de deux vecteurs u et v
//!< @param vecteur u
//!< @param vecteur v
//!< @return vecteur résultant de u+V
struct fx_vect2 fx_vect2_add(const struct fx_vect2* u, const struct fx_vect2* v);

//!< @function fx_vect2_difference
//!< @brief calcule la différence u-v de deux vecteurs u et v
//!< @param vecteur u
//!< @param vecteur v
//!< @return vecteur résultant de u-v
struct fx_vect2 fx_vect2_sub(const struct fx_vect2* u, const struct fx_vect2* v);

//!< @function fx_vect2_scalar_product
//!< @brief calcule le produit scalaire de deux vecteurs u et v
//!< @param vecteur u
//!< @param vecteur v
//!< @return produit scalaire calculé avec les positions en mm
int32_t fx_vect2_scalar_product(const struct fx_vect2* u, const struct fx_vect2* v);

//!< @function fx_vect2_vector_product_z
//!< @brief calcule la compose sur Oz du produit vectoriel de u et v (appartenant à (Ox,Ox))
//!< @param vecteur u
//!< @param vecteur v
//!< @return composante sur Oz du produit vectoriel calculé avec les positions en mm
int32_t fx_vect2_vector_product_z(const struct fx_vect2* u, const struct fx_vect2* v);

#endif
