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

#define COLOR_UNKNOWN     0
#define COLOR_RED         1
#define COLOR_YELLOW      2

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

#ifndef LINUX
static inline void gpio_color_change_disable()
{
	extern volatile uint8_t gpio_color_change_enable;
	gpio_color_change_enable = 0;
}

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

static inline uint8_t gpio_is_go_enable()
{
	extern volatile uint8_t gpio_enable_go;
	return gpio_enable_go;
}

uint32_t gpio_get_state();

void gpio_wait_go();
#endif
// ---------------- interface usb ------------
enum
{
	GPIO_CMD_ENABLE_GO,
	GPIO_CMD_GO,
};

struct gpio_cmd_go_arg
{
	uint8_t cmd;
};

#ifdef __cplusplus
}
#endif

#endif
