#ifndef TRIGO_H
#define TRIGO_H

//! @file trigo.h
//! @brief Fonctions trigo
//! @author Atlantronic

#include <stdint.h>

//! @param alpha en 2^-26 tours
//! @return sinus en 2^-30 unités
int32_t fx_sin(int32_t alpha);

//! @param alpha en 2^-26 tours
//! @return cosinus en 2^-30 unités
int32_t fx_cos(int32_t alpha);

//! @param x en 2^-16 mm
//! @param y en 2^-16 mm
//! @return arctan2 en 2^-26 tours
int32_t fx_atan2(int32_t y, int32_t x);

//! @param x en 2^-16 mm
//! @return arctan2 en 2^-26 tours
int32_t fx_acos(int32_t x);

//! @param x en 2^-16 unite
//! @retrun sqrt(x) en 2^-16 unite
uint32_t fx_sqrt(uint32_t x);

#endif
