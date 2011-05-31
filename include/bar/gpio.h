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

#define GPIO_US0     0x01  // PC1
#define GPIO_US1     0x02  // PC3
#define GPIO_US2     0x04  // PA5
#define GPIO_US3     0x08  // PC5
#define GPIO_US4     0x10  // PB1

void setLed(uint32_t mask);

uint16_t gpio_get_us(enum us_id us_id);

void gpio_send_us(uint8_t us_mask);

#endif
