#ifndef _GPIO_H
#define _GPIO_H

//! @file gpio.h
//! @brief Gpio mapping
//! @author Jean-Baptiste Trédez

#include "cpu/cpu.h"

#define COLOR_RED                 0
#define COLOR_BLUE    GPIO_IDR_IDR9

#define LED_0                  0x01
#define LED_1                  0x02
#define LED_2                  0x04
#define LED_3                  0x08
#define LED_4                  0x10
#define LED_5                  0x20

static inline int getColor()
{
	// pour les tests Linux on est bleu pour le moment
	return COLOR_BLUE;
}

static inline void setLed(uint8_t mask)
{
	(void) mask;
	// TODO
}

#endif
