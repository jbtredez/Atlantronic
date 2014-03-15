//! @file encoder.c
//! @brief Encoder
//! @author Atlantronic

#include "encoder.h"
#include "kernel/module.h"
#include "kernel/cpu/cpu.h"
#include "gpio.h"

static void encoder_init(GPIO_TypeDef* GPIOx_ch1, uint32_t pin_ch1, GPIO_TypeDef* GPIOx_ch2, uint32_t pin_ch2, TIM_TypeDef* tim, uint32_t gpio_af);

static int encoder_module_init()
{
	// activation GPIOB, GPIOC et GPIOD
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
	// activation du timer 2, 3 et 4
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;

	encoder_init(GPIOB, 4, GPIOB, 5, TIM3, GPIO_AF_TIM3); // encoder 1
	encoder_init(GPIOA, 15, GPIOB, 3, TIM2, GPIO_AF_TIM2); // encoder 2
	encoder_init(GPIOD, 12, GPIOD, 13, TIM4, GPIO_AF_TIM4); // encoder 3

	return 0;
}

module_init(encoder_module_init, INIT_ENCODERS);

static void encoder_init(GPIO_TypeDef* GPIOx_ch1, uint32_t pin_ch1, GPIO_TypeDef* GPIOx_ch2, uint32_t pin_ch2, TIM_TypeDef* tim, uint32_t gpio_af)
{
	gpio_pin_init(GPIOx_ch1, pin_ch1, GPIO_MODE_AF, GPIO_SPEED_100MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // TIMx_CH1
	gpio_pin_init(GPIOx_ch2, pin_ch2, GPIO_MODE_AF, GPIO_SPEED_100MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // TIMx_CH2
	gpio_af_config(GPIOx_ch1, pin_ch1, gpio_af);
	gpio_af_config(GPIOx_ch2, pin_ch2, gpio_af);

	tim->CR1 = 0x00;
	tim->CR2 = 0x00;

	// auto-reload au max
	tim->ARR = TIM_ARR_ARR;

	// on compte selon  les 2 lignes
	tim->SMCR = 0x00;
	tim->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;

	// pas d'inversions de polarité
	tim->CCER = 0x00;

	// Capture compare 1 et capture compare 2 en entrée
	tim->CCMR1 &= ~ (TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
	tim->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;

	// mise a zero
	tim->CNT = 0x00;

	// on active le tout
	tim->CR1 |= TIM_CR1_CEN;
}

uint16_t encoder_get(const unsigned int num)
{
	uint16_t val;
	switch(num)
	{
		case ENCODER_1:
			val = TIM3->CNT;
			break;
		case ENCODER_2:
			val = TIM2->CNT;
			break;
		case ENCODER_3:
			val = TIM4->CNT;
			break;
		default:
			// TODO log erreur
			val = 0;
			break;
	}

	return val;
}

