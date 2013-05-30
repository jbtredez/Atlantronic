#ifndef GPIO_H
#define GPIO_H

//! @file gpio.h
//! @brief Gpio mapping
//! @author Atlantronic

#include "kernel/cpu/cpu.h"

#define LED_GREEN    0x1000
#define LED_ORANGE   0x2000
#define LED_RED      0x4000
#define LED_BLUE     0x8000

void setLed(uint16_t mask);

#endif
