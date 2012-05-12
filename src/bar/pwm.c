//! @file pwm.c
//! @brief PWM
//! @author Atlantronic

#include "pwm.h"
#include "kernel/module.h"
#include "kernel/cpu/cpu.h"
#include "kernel/rcc.h"
#include "kernel/robot_parameters.h"

void isr_pwm_reset(void);

static int pwm_module_init()
{
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

	// TIM1 full remap => CH1/4 : PE9, PE11, PE13, PE14
	AFIO->MAPR &= ~AFIO_MAPR_TIM1_REMAP;
	AFIO->MAPR |= AFIO_MAPR_TIM1_REMAP_FULLREMAP;

	// GPIOE utilisee :
	// --> configuration sorties pwm : PE14 => alternate output push-pull, 50MHz
	RCC->APB2ENR |=  RCC_APB2ENR_IOPEEN;
	GPIOE->CRH = ( GPIOE->CRH & ~( GPIO_CRH_MODE14 | GPIO_CRH_CNF14 // on efface la conf de PE14
	             )) |
	             GPIO_CRH_CNF14_1 | GPIO_CRH_MODE14_0 | GPIO_CRH_MODE14_1 ; // PE14 : alternate output push-pull, 50MHz

	// activation clock sur le timer 1
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	TIM1->PSC = PWM_PSC;
	TIM1->ARR = PWM_ARR;

	TIM1->RCR =0x00;
	TIM1->CR1 = 0x00;
	TIM1->CR2 = 0x00;

	TIM1->CR1 |= /*TIM_CR1_ARPE |*/ TIM_CR1_URS;
	TIM1->SMCR = 0x00;

	// mise à jour "update generation"
	TIM1->EGR |= TIM_EGR_UG;

	TIM1->CCER = 0x00; // permet de programmer CCMR1 et CCMR2
	TIM1->BDTR = 0x00; // permet de programmer CCMR1 et CCMR2
	TIM1->CCMR2 = TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;  // mode PWM 1 sur le canal 4 avec preload

	TIM1->CCER = TIM_CCER_CC4E;

	TIM1->CCR4 = PWM_SERVO1_MIN; // pwm initiale sur le canal 4 (position du servo au milieu)

	// on active le tout
	TIM1->CR1 |= TIM_CR1_CEN;
	TIM1->BDTR |= TIM_BDTR_MOE;

	return 0;
}

module_init(pwm_module_init, INIT_PWM);

void pwm_set(const unsigned int num, int16_t val)
{
	// min = (INIT_PWM * 2.7f)/100
	// max = (INIT_PWM * 12.1f)/100
	// milieu = (INIT_PWM * 7.1f)/100
	switch(num)
	{
		case PWM_SERVO1:
			if(val < PWM_SERVO1_MIN)
			{
				val = PWM_SERVO1_MIN;
			}
			else if(val > PWM_SERVO1_MAX)
			{
				val = PWM_SERVO1_MAX;
			}
			TIM1->CCR4 = val;
			break;
		default:
			// TODO : log erreur
			break;
	}
}

void isr_pwm_reset(void)
{
	// on est dans une IT d'erreur, tout va mal => arrêt des moteurs
	// laisser la pwm du servo a sa valeur
}
