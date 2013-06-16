//! @file vect2.c
//! @brief vect2
//! @author Atlantronic

#include "kernel/math/vect2.h"

//!< @function fx_vect2_sum
//!< @brief calcule la somme de deux vecteurs u et v
//!< @param vecteur u
//!< @param vecteur v
//!< @return vecteur résultant de u+v
struct fx_vect2 fx_vect2_add(const struct fx_vect2* u, const struct fx_vect2* v)
{
	struct fx_vect2 result = {u->x + v->x, u->y + v->y}; 
	return result;
}

//!< @function fx_vect2_difference
//!< @brief calcule la différence u-v de deux vecteurs u et v
//!< @param vecteur u
//!< @param vecteur v
//!< @return vecteur résultant de u-v
struct fx_vect2 fx_vect2_sub(const struct fx_vect2* u, const struct fx_vect2* v)
{
	struct fx_vect2 result = {u->x - v->x, u->y - v->y}; 
	return result;
}

//!< @function fx_vect2_scalar_product
//!< @brief calcul le produit scalaire de deux vecteurs u et v
//!< @param vecteur u
//!< @param vecteur v
//!< @return produit scalaire calculé avec les positions en mm
int32_t fx_vect2_scalar_product(const struct fx_vect2* u, const struct fx_vect2* v)
{
	return ((u->x>>16)*(v->x>>16))+((u->y>>16)*(v->y>>16));
}

//!< @function fx_vect2_vector_product_z
//!< @brief calcule la compose sur Oz du produit vectoriel de u et v (appartenant à (Ox,Ox))
//!< @param vecteur u
//!< @param vecteur v
//!< @return composante sur Oz du produit vectoriel calculé avec les positions en mm
int32_t fx_vect2_vector_product_z(const struct fx_vect2* u, const struct fx_vect2* v)
{
	return ((u->x>>16)*(v->y>>16))-((u->y>>16)*(v->x>>16));
}

