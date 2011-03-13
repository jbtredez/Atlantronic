//! @file encoders.c
//! @brief Encoders
//! @author Jean-Baptiste Trédez

#include "io/encoders.h"
#include "module.h"
#include "init.h"
#include "cpu/cpu.h"

static int encoders_module_init()
{
	// Timer 2 pas remape : PA0, PA1, PA2, PA3
	AFIO->MAPR &= ~AFIO_MAPR_TIM2_REMAP;

	// GPIOA utilisee, configuration de PA0 et PA1
	RCC->APB2ENR |=  RCC_APB2ENR_IOPAEN;
	GPIOA->CRL   &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0 | GPIO_CRL_MODE1 | GPIO_CRL_CNF1);     // on efface la conf de PA0 et PA1

	// activation de la clock sur le timer 2
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	TIM2->CR1 = 0x00;
	TIM2->CR2 = 0x00;
	// auto-reload au max
	TIM2->ARR = TIM_ARR_ARR;

	// on compte selon  les 2 lignes
	TIM2->SMCR = 0x00;
	TIM2->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;

	// Capture compare 1 et capture compare 2 en entrée
	TIM2->CCMR1 &= ~ (TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
	TIM2->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;

	// pas d'inversions de polarité
	TIM2->CCER = 0x00;

	// mise a zero
	TIM2->CNT = 0x00;

	// on active le tout
	TIM2->CR1 |= TIM_CR1_CEN;

	// activation de la clock sur AFIO
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

	// Timer 4 : remap : PD12, PD13, PD14, PD15
	AFIO->MAPR |= AFIO_MAPR_TIM4_REMAP;

	// GPIOD utilisee, configuration de PD12 et PD13
	RCC->APB2ENR |=  RCC_APB2ENR_IOPDEN;
	GPIOA->CRH   &= ~(GPIO_CRH_MODE12 | GPIO_CRH_CNF12 | GPIO_CRH_MODE13 | GPIO_CRH_CNF13);     // on efface la conf de PD12 et PD13
	GPIOA->CRH   |=  GPIO_CRH_CNF12_1;               // PD12 : input pull-up / pull-down
	GPIOA->CRH   |=  GPIO_CRH_CNF13_1;               // PD13 : input pull-up / pull-down

	// activation de la clock sur le timer 4
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

	TIM4->CR1 = 0x00;
	TIM4->CR2 = 0x00;
	// auto-reload au max
	TIM4->ARR = TIM_ARR_ARR;

	// on compte selon  les 2 lignes
	TIM4->SMCR = 0x00;
	TIM4->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;

	// Capture compare 1 et capture compare 2 en entrée
	TIM4->CCMR1 &= ~ (TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
	TIM4->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;

	// pas d'inversions de polarité
	TIM4->CCER = 0x00;

	// mise a zero
	TIM4->CNT = 0x00;

	// on active le tout
	TIM4->CR1 |= TIM_CR1_CEN;

	return 0;
}

module_init(encoders_module_init, INIT_ENCODERS);

uint16_t encoders_get(unsigned int num)
{
	uint16_t val;
	if(num == ENCODERS_MOT_RIGHT)
	{
		val = TIM2->CNT;
	}
	else
	{
		val = TIM4->CNT;
	}

	return val;
}

