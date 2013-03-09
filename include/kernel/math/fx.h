#ifndef FX_H
#define FX_H

//! @file fx.h
//! @brief Macros de conversions vers unités fx
//! @author Atlantronic

#define mm2fx(a)         (((int32_t)a) << 16)
#define fx2mm(a)         (((int32_t)a) >> 16)

#endif
