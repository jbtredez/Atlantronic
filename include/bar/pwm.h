#ifndef PWM_H
#define PWM_H

//! @file pwm.h
//! @brief PWM interface
//! @author Atlantronic

#include <stdint.h>
#include "kernel/rcc.h"

#define PWM_SERVO1          0
#define PWM_SERVO_BALISE    1

#if( RCC_PCLK2 != 72000000)
#error revoir les pwm
#endif

// PSC = (RCC_PCLK2 / TIM1CLK) - 1 si RCC_PCLK2 == HCLK
// PSC = (RCC_PCLK2 * 2 / TIM1CLK) - 1 sinon
// But :
//  - profiter un max de la plage des 16bits pour la PWM
// => PSC = 22, on maximise TIMCLK donc ARR
#define PWM_PSC        21

// pour PSC = 0xFFFF, TIMCLK = 3272727 Hz
// donc ARR = TIM1CLK / 50 - 1 = 65453 pour une PWM à 50 Hz. On a une resolution de 65453 sur une periode
#define PWM_ARR     65453

#define PWM_SERVO1_MIN    (int16_t)((PWM_ARR * 2.7f)/100)
#define PWM_SERVO1_MAX    (int16_t)((PWM_ARR * 12.1f)/100)
#define PWM_SERVO1_MED    (int16_t)((PWM_ARR * 7.1f)/100)

//! set a pwm
//!
//! @param num pwm id
//! @param val new value
void pwm_set(const unsigned int num, int16_t val);

#endif
