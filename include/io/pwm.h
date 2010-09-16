#ifndef PWM_H
#define PWM_H

//! @file pwm.h
//! @brief PWM
//! @author Jean-Baptiste Trédez

#include <stdint.h>

//! number of pwm
#define PWM_NB        2

#define PWM_RIGHT    0
#define PWM_LEFT     1

//! set a pwm
//!
//! @param num pwm id
//! @param val new value
void pwm_set(unsigned int num, uint32_t val, int dir);

#endif
