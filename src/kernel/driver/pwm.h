#ifndef PWM_H
#define PWM_H

//! @file pwm.h
//! @brief PWM interface
//! @author Atlantronic

#include <stdint.h>
#include "kernel/rcc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PWM_PUMP_1     0
#define PWM_PUMP_2     1
#define PWM_PUMP_3     2
#define PWM_PUMP_4     3

#if( RCC_PCLK2_MHZ != 84)
#error revoir les pwm
#endif

// PSC = (RCC_PCLK2 / TIM1CLK) - 1
// But :
//  - profiter un max de la plage des 16bits pour la PWM
// => PSC = 0, on maximise TIMCLK (= PCLK) donc ARR
#define PWM_PSC     0x00

// pour PSC = 0, TIMCLK = 84 MHz
// donc ARR = TIM1CLK / 25000 - 1 = 3359 pour une PWM à 25 kHz. On a une resolution de 3360 sur une periode
#define PWM_ARR     3359

//! set a pwm
//!
//! @param num pwm id
//! @param val new value ([-PWM_ARR ; PWM_ARR])
void pwm_set16(const unsigned int num, int16_t val);


//! set a pwm
//!
//! @param num pwm id
//! @param val new value ([-1 ; 1])
void pwm_set(const unsigned int num, float val);


#ifdef __cplusplus
}
#endif

#endif
