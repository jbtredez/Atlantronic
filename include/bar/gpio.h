#ifndef GPIO_H
#define GPIO_H

//! @file gpio.h
//! @brief Gpio mapping
//! @author Atlantronic

#include "kernel/cpu/cpu.h"
#include "kernel/can/can_us.h"

#define COLOR_RED                 0
#define COLOR_BLUE                1

#define LED_0                  0x01
#define LED_1                  0x02
#define LED_2                  0x04
#define LED_3                  0x08
#define LED_4                  0x10
#define LED_5                  0x20
#define LED_WARNING           0x200

void setLed(uint32_t mask);

static inline uint16_t get_US(enum us_id us_id)
{
      //TODO
	return us_id;
}

#endif
