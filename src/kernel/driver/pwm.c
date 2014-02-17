//! @file pwm.c
//! @brief PWM
//! @author Atlantronic

#include "pwm.h"
#include "gpio.h"
#include "kernel/module.h"
#include "kernel/cpu/cpu.h"

void isr_pwm_reset(void);

static int pwm_module_init()
{
	// activation GPIOE
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

	gpio_pin_init(GPIOE, 9, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // TIM1_CH1
	gpio_pin_init(GPIOE, 11, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // TIM1_CH2
	gpio_pin_init(GPIOE, 13, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // TIM1_CH3
	gpio_pin_init(GPIOE, 14, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // TIM1_CH4
	gpio_af_config(GPIOE, 9, GPIO_AF_TIM1);
	gpio_af_config(GPIOE, 11, GPIO_AF_TIM1);
	gpio_af_config(GPIOE, 13, GPIO_AF_TIM1);
	gpio_af_config(GPIOE, 14, GPIO_AF_TIM1);

	gpio_pin_init(GPIOE, 8, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // sens 1
	gpio_pin_init(GPIOE, 10, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // sens 2
	gpio_pin_init(GPIOE, 12, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // sens 3
	gpio_pin_init(GPIOE, 15, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP); // sens 4

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

module_exit(isr_pwm_reset, EXIT_PWM);

void pwm_set16(const unsigned int num, int16_t val)
{
	int dir = 1;
	if(val < 0)
	{
		val = -val;
		dir = -1;
	}

	// saturation
	if( val > PWM_ARR )
	{
		val = PWM_ARR;
	}

	switch(num)
	{
		case 0:
			if(dir > 0)
			{
				gpio_set_pin(GPIOE, 8);
			}
			else
			{
				gpio_reset_pin(GPIOE, 8);
			}
			TIM1->CCR1 = val;
			break;
		case 1:
			if(dir > 0)
			{
				gpio_set_pin(GPIOE, 10);
			}
			else
			{
				gpio_reset_pin(GPIOE, 10);
			}
			TIM1->CCR2 = val;
			break;
		case 2:
			if(dir > 0)
			{
				gpio_set_pin(GPIOE, 12);
			}
			else
			{
				gpio_reset_pin(GPIOE, 12);
			}
			TIM1->CCR3 = val;
			break;
		case 3:
			if(dir > 0)
			{
				gpio_set_pin(GPIOE, 15);
			}
			else
			{
				gpio_reset_pin(GPIOE, 15);
			}
			TIM1->CCR4 = val;
			break;
		default:
			// TODO : log erreur
			break;
	}
}

void pwm_set(const unsigned int num, float val)
{
	pwm_set16(num, val * PWM_ARR);
}

void isr_pwm_reset(void)
{
	// on est dans une IT d'erreur ou fin du match => arrêt des moteurs
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR4 = 0;
}
