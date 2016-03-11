//! @file encoder.c
//! @brief Encoder
//! @author Atlantronic

#include "EncoderAB.h"
#include "kernel/module.h"
#include "kernel/cpu/cpu.h"
#include "kernel/driver/gpio.h"

static volatile uint32_t* encoder_cnt[ENCODER_MAX];

static void encoder_ab_init(const unsigned int id, GPIO_TypeDef* GPIOx_ch1, uint32_t pin_ch1, GPIO_TypeDef* GPIOx_ch2, uint32_t pin_ch2, TIM_TypeDef* tim, uint32_t gpio_af);

static int encoder_ab_module_init()
{
	// activation GPIOA, GPIOB, GPIOC et GPIOD
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
	// activation du timer 2, 3 et 4
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;

	encoder_ab_init(ENCODER_1, GPIOA, 0, GPIOA, 1, TIM2, GPIO_AF_TIM2); // encoder 1
	encoder_ab_init(ENCODER_2, GPIOD, 12, GPIOB, 7, TIM4, GPIO_AF_TIM4); // encoder 2
	encoder_ab_init(ENCODER_3, GPIOB, 4, GPIOC, 7, TIM3, GPIO_AF_TIM3); // encoder 3

	return 0;
}

module_init(encoder_ab_module_init, INIT_ENCODERS);

static void encoder_ab_init(const unsigned int id, GPIO_TypeDef* GPIOx_ch1, uint32_t pin_ch1, GPIO_TypeDef* GPIOx_ch2, uint32_t pin_ch2, TIM_TypeDef* tim, uint32_t gpio_af)
{
	if( id >= ENCODER_MAX )
	{
		return;
	}

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

	encoder_cnt[id] = &tim->CNT;
}

uint16_t encoder_ab_get(const unsigned int id)
{
	uint16_t val = 0;

	if( id < ENCODER_MAX )
	{
		val = *encoder_cnt[id];
	}

	return val;
}

void EncoderAB::init(int id, float outputFactor)
{
	m_id = id;
	m_outputFactor = outputFactor;
	m_rawPos = encoder_ab_get(m_id);
	m_lastRawPos = m_rawPos;
}

void EncoderAB::update(float dt)
{
	m_lastRawPos = m_rawPos;
	m_rawPos = encoder_ab_get(m_id);
	// attention aux types, soustraction de 2 uint16_t qu'on met dans un int16_t
	int16_t m_rawSpeed = (int16_t)(m_rawPos - m_lastRawPos);
	m_speed = ((float)m_rawSpeed) * m_outputFactor / dt;
}

float EncoderAB::getPosition()
{
	return m_rawPos * m_outputFactor;
}

float EncoderAB::getSpeed()
{
	return m_speed;
}
