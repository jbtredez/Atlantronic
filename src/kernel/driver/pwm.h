#ifndef PWM_H
#define PWM_H

//! @file pwm.h
//! @brief PWM interface
//! @author Atlantronic

#include <stdint.h>
#include "kernel/rcc.h"
#include "kernel/log.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef WEAK_PWM
#define WEAK_PWM __attribute__((weak, alias("nop_function") ))
#endif

enum
{
	PWM_1,
	PWM_2,
	PWM_3,
	PWM_4,
	PWM_MAX,
};

#if( RCC_PCLK1_MHZ != 42)
#error revoir les pwm
#endif

#if( RCC_PCLK2_MHZ != 84)
#error revoir les pwm
#endif

// PSC = (RCC_PCLK2 / TIM1CLK) - 1
// But :
//  - profiter un max de la plage des 16bits pour la PWM
// => PSC = 0, on maximise TIMCLK (= PCLK) donc ARR
#define PWM_PSC     0x00

// pour PSC = 0, TIMCLK = 84 MHz,
// ARR = TIM1CLK / 25000 - 1 = 3359 pour une PWM à 25 kHz. On a une resolution de 3360 sur une periode
// pour PSC = 0, TIMCLK = 42 MHz,
// ARR = TIM1CLK / 25000 - 1 = 1679 pour une PWM à 25 kHz. On a une resolution de 1680 sur une periode
#define PWM_ARR1     3359
#define PWM_ARR2     1679

//! set a pwm
//!
//! @param id pwm id
//! @param val new value ([-1 ; 1])
void pwm_set(const unsigned int id, float val);

//! enable pwm
void pwm_enable() WEAK_PWM;

//! disable pwm (power off)
void pwm_disable() WEAK_PWM;

#ifdef __cplusplus
}
#endif

#endif
