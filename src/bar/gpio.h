#ifndef GPIO_H
#define GPIO_H

//! @file gpio.h
//! @brief Gpio mapping
//! @author Atlantronic

#include "kernel/cpu/cpu.h"
#include "kernel/us_def.h"

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

void gpio_get_us(uint16_t* us_distance, uint8_t size);

void gpio_send_us(uint8_t us_mask);

void gpio_activate_right_us();

void gpio_activate_left_us();

#endif
