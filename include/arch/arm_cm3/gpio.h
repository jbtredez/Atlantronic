#ifndef _GPIO_H
#define _GPIO_H

//! @file gpio.h
//! @brief Gpio mapping
//! @author Jean-Baptiste Trédez

#include "cpu/cpu.h"

#define COLOR_RED                 0
#define COLOR_BLUE    GPIO_IDR_IDR9

static inline int getColor()
{
	return GPIOD->IDR & COLOR_BLUE;
}

#endif
