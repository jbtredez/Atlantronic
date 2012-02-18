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
	// --> configuration sorties pwm   : PE9, PE11, PE13 et PE14 => alternate output push-pull, 50MHz
	// --> configuration patte de sens : PE8, PE10, PE12 et PE15 => output push-pull, 50MHz
	RCC->APB2ENR |=  RCC_APB2ENR_IOPEEN;
	GPIOE->CRH = ( GPIOE->CRH & ~( GPIO_CRH_MODE8  | GPIO_CRH_CNF8  | // on efface la conf de PE8
	                               GPIO_CRH_MODE9  | GPIO_CRH_CNF9  | // on efface la conf de PE9
	                               GPIO_CRH_MODE10 | GPIO_CRH_CNF10 | // on efface la conf de PE10
	                               GPIO_CRH_MODE11 | GPIO_CRH_CNF11 | // on efface la conf de PE11
	                               GPIO_CRH_MODE12 | GPIO_CRH_CNF12 | // on efface la conf de PE12
	                               GPIO_CRH_MODE13 | GPIO_CRH_CNF13 | // on efface la conf de PE13
	                               GPIO_CRH_MODE14 | GPIO_CRH_CNF14 | // on efface la conf de PE14
	                               GPIO_CRH_MODE15 | GPIO_CRH_CNF15   // on efface la conf de PE15
	             )) |
	                                GPIO_CRH_MODE8_0  | GPIO_CRH_MODE8_1  | // PE8  : output push-pull, 50MHz
	             GPIO_CRH_CNF9_1  | GPIO_CRH_MODE9_0  | GPIO_CRH_MODE9_1  | // PE9  : alternate output push-pull, 50MHz
	                                GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1 | // PE10 : output push-pull, 50MHz
	             GPIO_CRH_CNF11_1 | GPIO_CRH_MODE11_0 | GPIO_CRH_MODE11_1 | // PE11 : alternate output push-pull, 50MHz
	                                GPIO_CRH_MODE12_0 | GPIO_CRH_MODE12_1 | // PE12 : output push-pull, 50MHz
	             GPIO_CRH_CNF13_1 | GPIO_CRH_MODE13_0 | GPIO_CRH_MODE13_1 | // PE13 : alternate output push-pull, 50MHz
	             GPIO_CRH_CNF14_1 | GPIO_CRH_MODE14_0 | GPIO_CRH_MODE14_1 | // PE14 : alternate output push-pull, 50MHz
	                                GPIO_CRH_MODE15_0 | GPIO_CRH_MODE15_1;  // PE15 : output push-pull, 50MHz

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
	TIM1->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE | // mode PWM 1 sur le canal 1 avec preload
	              TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;  // mode PWM 1 sur le canal 2 avec preload
	TIM1->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE | // mode PWM 1 sur le canal 3 avec preload
	              TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;  // mode PWM 1 sur le canal 4 avec preload

	TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

	TIM1->CCR1 = 0x00; // pwm initiale a 0 sur le canal 1
	TIM1->CCR2 = 0x00; // pwm initiale a 0 sur le canal 2
	TIM1->CCR3 = 0x00; // pwm initiale a 0 sur le canal 3
	TIM1->CCR4 = 0x00; // pwm initiale a 0 sur le canal 4

	// on active le tout
	TIM1->CR1 |= TIM_CR1_CEN;
	TIM1->BDTR |= TIM_BDTR_MOE;

	return 0;
}

module_init(pwm_module_init, INIT_PWM);

void pwm_set(const unsigned int num, int16_t val)
{
	int dir = 1;
	if(val < 0)
	{
		val = -val;
		dir = -1;
	}

	switch(num)
	{
		case PWM_RIGHT:
			if(dir*PARAM_RIGHT_MOT_WHEEL_WAY > 0)
			{
				GPIOE->ODR |= GPIO_ODR_ODR8;
			}
			else
			{
				GPIOE->ODR &= ~GPIO_ODR_ODR8;
			}
			TIM1->CCR1 = val;
			break;
		case PWM_LEFT:
			if(dir*PARAM_LEFT_MOT_WHEEL_WAY > 0)
			{
				GPIOE->ODR |= GPIO_ODR_ODR10;
			}
			else
			{
				GPIOE->ODR &= ~GPIO_ODR_ODR10;
			}
			TIM1->CCR2 = val;
			break;
		case PWM_UP_RIGHT:
			if(dir > 0)
			{
				GPIOE->ODR |= GPIO_ODR_ODR12;
			}
			else
			{
				GPIOE->ODR &= ~GPIO_ODR_ODR12;
			}
			TIM1->CCR3 = val;
			break;
		case PWM_UP_LEFT:
			if(dir > 0)
			{
				GPIOE->ODR |= GPIO_ODR_ODR15;
			}
			else
			{
				GPIOE->ODR &= ~GPIO_ODR_ODR15;
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
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR4 = 0;
}
