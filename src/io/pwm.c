//! @file pwm.c
//! @brief PWM
//! @author Jean-Baptiste Trédez

#include "io/pwm.h"
#include "module.h"
#include "init.h"
#include "cpu/cpu.h"
#include "io/rcc.h"
#include "robot_parameters.h"

static int pwm_module_init()
{
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

	// TIM1 full remap => CH1/4 : PE9, PE11, PE13, PE14
	AFIO->MAPR &= ~AFIO_MAPR_TIM1_REMAP;
	AFIO->MAPR |= AFIO_MAPR_TIM1_REMAP_FULLREMAP;

	// GPIOE utilisee, configuration de PE9 et PE11
	RCC->APB2ENR |=  RCC_APB2ENR_IOPEEN;
	GPIOE->CRH   &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9 | GPIO_CRH_MODE11 | GPIO_CRH_CNF11);     // on efface la conf de PE9 et PE11
	GPIOE->CRH   |=  GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1;               	  // PE9  : alternate output push-pull, 50MHz
	GPIOE->CRH   |=  GPIO_CRH_CNF11_1 | GPIO_CRH_MODE11_0 | GPIO_CRH_MODE11_1;                // PE11 : alternate output push-pull, 50MHz

	// conf patte de sens : PE8 et PE10
	GPIOE->CRH   &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8 | GPIO_CRH_MODE10 | GPIO_CRH_CNF10);     // on efface la conf de PE8 et PE10
	GPIOE->CRH   |=  GPIO_CRH_MODE8_0 | GPIO_CRH_MODE8_1;                  // PE8  : output push-pull, 50MHz
	GPIOE->CRH   |=  GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1;                // PE10 : output push-pull, 50MHz

	// activation clock sur le timer 1
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	#if( RCC_PCLK2 != 72000000)
	#error TIM1->PSC à recalculer
	#endif
	// PSC = (RCC_PCLK2 / TIM1CLK) - 1 si RCC_PCLK2 == HCLK
	// PSC = (RCC_PCLK2 * 2 / TIM1CLK) - 1 sinon

	// But :
	//  - profiter un max de la plage des 16bits pour la PWM
	//  - PWM a environ 40 kHz
	// pour PSC = 0, TIM1CLK = 72 MHz
	// donc ARR = TIM1CLK / 40000 - 1 = 1799 pour une PWM à 40 kHz. On a une resolution de 1800 sur une periode

	TIM1->PSC = 0x00;
	TIM1->ARR = 1799;

	TIM1->RCR =0x00;
	TIM1->CR1 = 0x00;
	TIM1->CR2 = 0x00;

	TIM1->CR1 |= /*TIM_CR1_ARPE |*/ TIM_CR1_URS;
	TIM1->SMCR = 0x00;

	// mise à jour "update generation"
	TIM1->EGR |= TIM_EGR_UG;

	TIM1->CCER = 0x00;
	TIM1->CCMR1 = 0x00;
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE; // mode PWM 1 sur le canal 1 avec preload
	TIM1->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE; // mode PWM 1 sur le canal 2 avec preload
	TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;

	TIM1->CCR1 = 0x00; // pwm initiale a 0 sur le canal 1
	TIM1->CCR2 = 0x00; // pwm initiale a 0 sur le canal 2

	// on active le tout
	TIM1->CR1 |= TIM_CR1_CEN;
	TIM1->BDTR |= TIM_BDTR_MOE;

// TODO PWM_UP_*

	return 0;
}

module_init(pwm_module_init, INIT_PWM);

void pwm_set(const unsigned int num, uint16_t val, int dir)
{
// TODO PWM_UP_*
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
		default:
			// TODO : log erreur
			break;
	}
}

