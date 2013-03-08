#ifndef PWM_H
#define PWM_H

//! @file pwm.h
//! @brief PWM interface
//! @author Atlantronic

#include <stdint.h>
#include "kernel/rcc.h"

#define PWM_RIGHT    0
#define PWM_LEFT     1
#define PWM_BRIDGE   2
#define PWM_FAN      3

#if( RCC_PCLK2 != 72000000)
#error revoir les pwm
#endif

// PSC = (RCC_PCLK2 / TIM1CLK) - 1 si RCC_PCLK2 == HCLK
// PSC = (RCC_PCLK2 * 2 / TIM1CLK) - 1 sinon
// But :
//  - profiter un max de la plage des 16bits pour la PWM
// => PSC = 0, on maximise TIMCLK donc ARR
#define PWM_PSC     0x00

// pour PSC = 0, TIMCLK = 72 MHz
// donc ARR = TIM1CLK / 25000 - 1 = 2879 pour une PWM à 25 kHz. On a une resolution de 2880 sur une periode
#define PWM_ARR     2879

//! set a pwm
//!
//! @param num pwm id
//! @param val new value
void pwm_set(const unsigned int num, int16_t val);

#endif
