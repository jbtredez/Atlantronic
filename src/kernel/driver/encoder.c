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
	// activation GPIOB et GPIOE
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOEEN;
	// activation du timer 3, 9 et 12
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM12EN;
	RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;

	encoder_init(GPIOB, 4, GPIOB, 5, TIM3, GPIO_AF_TIM3); // encoder 1
	encoder_init(GPIOB, 14, GPIOB, 15, TIM12, GPIO_AF_TIM12); // encoder 2
	encoder_init(GPIOE, 5, GPIOE, 6, TIM12, GPIO_AF_TIM9); // encoder 3

	return 0;
}

module_init(encoder_module_init, INIT_ENCODERS);

static void encoder_init(GPIO_TypeDef* GPIOx_ch1, uint32_t pin_ch1, GPIO_TypeDef* GPIOx_ch2, uint32_t pin_ch2, TIM_TypeDef* tim, uint32_t gpio_af)
{
	gpio_pin_init(GPIOx_ch1, pin_ch1, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // TIMx_CH1
	gpio_pin_init(GPIOx_ch2, pin_ch2, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // TIMx_CH2
	gpio_af_config(GPIOx_ch1, pin_ch1, gpio_af);
	gpio_af_config(GPIOx_ch2, pin_ch2, gpio_af);

	tim->CR1 = 0x00;
	tim->CR2 = 0x00;

	// auto-reload au max
	tim->ARR = TIM_ARR_ARR;

	// on compte selon  les 2 lignes
	tim->SMCR = 0x00;
	tim->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;

	// Capture compare 1 et capture compare 2 en entrée
	tim->CCMR1 &= ~ (TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
	tim->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;

	// pas d'inversions de polarité
	tim->CCER = 0x00;

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
			val = TIM12->CNT;
			break;
		case ENCODER_3:
			val = TIM9->CNT;
			break;
		default:
			// TODO log erreur
			val = 0;
			break;
	}

	return val;
}

