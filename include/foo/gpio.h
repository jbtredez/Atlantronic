#ifndef GPIO_H
#define GPIO_H

//! @file gpio.h
//! @brief Gpio mapping
//! @author Atlantronic

#include "kernel/cpu/cpu.h"

#define COLOR_RED                 0
#define COLOR_BLUE                1

#define LED_0                  0x01
#define LED_1                  0x02
#define LED_2                  0x04
#define LED_3                  0x08
#define LED_4                  0x10
#define LED_5                  0x20
#define LED_WARNING           0x200

#define CONTACT_RIGHT          0x01
#define CONTACT_LEFT           0x02

static inline uint32_t getcolor()
{
	extern volatile uint32_t color;
	return color;
}

static inline uint8_t getGo()
{
	extern volatile uint8_t gpio_go;
	return gpio_go;
}

static inline uint8_t getRecalage()
{
	extern volatile uint8_t gpio_recaler;
	return gpio_recaler;
}

static inline void resetRecalage()
{
	extern volatile uint8_t gpio_recaler;
	gpio_recaler = 0;
}

void setLed(uint32_t mask);

static inline uint8_t get_contact()
{
	return GPIOB->IDR & (CONTACT_RIGHT | CONTACT_LEFT);
}

#endif
