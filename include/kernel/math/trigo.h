#ifndef TRIGO_H
#define TRIGO_H

//! @file trigo.h
//! @brief Fonctions trigo
//! @author Atlantronic

#include <stdint.h>

//!< @param alpha en 2^-24 tours
//!< @return sinus en 2^-30 unités
int32_t fx_sin(int32_t alpha);

//!< @param alpha en 2^-24 tours
//!< @return cosinus en 2^-30 unités
int32_t fx_cos(int32_t alpha);

#endif