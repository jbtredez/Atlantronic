#ifndef _GPIO_H
#define _GPIO_H

//! @file gpio.h
//! @brief Gpio mapping
//! @author Jean-Baptiste Trédez

#include "cpu/cpu.h"

#define COLOR_RED                 0
#define COLOR_BLUE                1

#define LED_0                  0x01
#define LED_1                  0x02
#define LED_2                  0x04
#define LED_3                  0x08
#define LED_4                  0x10
#define LED_5                  0x20
#define LED_WARNING           0x200

extern volatile uint32_t color;

static inline void setLed(uint32_t mask)
{
	GPIOE->ODR = (GPIOE->ODR & ~((uint32_t)0x3F)) | (mask & 0x3F);
	GPIOB->ODR = (GPIOB->ODR & ~((uint32_t)LED_WARNING)) | (mask & LED_WARNING);
}

#endif
