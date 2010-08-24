#ifndef PWM_H
#define PWM_H

//! @file pwm.h
//! @brief Encoders
//! @author Jean-Baptiste Trédez

#include <stdint.h>

//! set a pwm
//!
//! @param num pwm id
//! @param val new value
void pwm_set(int num, uint32_t val);

#endif
