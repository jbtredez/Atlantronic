#ifndef BSP_H
#define BSP_H

//! @file bsp.h
//! @brief BSP
//! @author Atlantronic

#include "kernel/cpu/cpu.h"
#include "kernel/driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GPIO_IN_1          0x01  // IN_1
#define GPIO_IN_2          0x02  // IN_2
#define GPIO_IN_3          0x04  // IN_3
#define GPIO_IN_4          0x08  // IN_4
#define GPIO_IN_5          0x10  // IN_5
#define GPIO_IN_6          0x20  // IN_6
#define GPIO_IN_7          0x40  // IN_7
#define GPIO_IN_8          0x80  // IN_8
#define GPIO_IN_9         0x100  // IN_9
#define GPIO_IN_10        0x200  // IN_10
#define GPIO_IN_11        0x400  // IN_11
#define GPIO_IN_12        0x800  // IN_12
#define GPIO_IN_13       0x1000  // IN_13
#define GPIO_IN_14       0x2000  // IN_14
#define GPIO_IN_BTN1     0x4000  // IN_BTN1 (etat pin)
#define GPIO_IN_BTN2     0x8000  // IN_BTN2 (etat pin)
#define GPIO_IN_GO      0x10000  // IN_GO (etat pin)
#define GPIO_GO         0x20000  // GO : match lance

uint32_t gpio_get_state();

#ifdef __cplusplus
}
#endif

#endif
