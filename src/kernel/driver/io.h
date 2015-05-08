#ifndef BSP_H
#define BSP_H

//! @file bsp.h
//! @brief BSP
//! @author Atlantronic

#include <stdint.h>

typedef enum io
{
	GPIO_0 = 0, // IO_0
	GPIO_1,     // IO_1
	GPIO_2,     // IO_2
	GPIO_3,     // IO_3
	GPIO_4,     // IO_4
	GPIO_5,     // IO_5
	GPIO_6,     // IO_6
	GPIO_7,     // IO_7
	GPIO_8,     // IO_8
	GPIO_9,     // IO_9
	GPIO_10,    // IO_10
	GPIO_11,    // IO_11
	GPIO_IN_GO, // IO_GO (etat pin)
} Io;

#define GPIO_MASK_0          0x01 // IO_0
#define GPIO_MASK_1          0x02 // IO_1
#define GPIO_MASK_2          0x04 // IO_2
#define GPIO_MASK_3          0x08 // IO_3
#define GPIO_MASK_4          0x10 // IO_4
#define GPIO_MASK_5          0x20 // IO_5
#define GPIO_MASK_6          0x40 // IO_6
#define GPIO_MASK_7          0x80 // IO_7
#define GPIO_MASK_8         0x100 // IO_8
#define GPIO_MASK_9         0x200 // IO_9
#define GPIO_MASK_10        0x400 // IO_10
#define GPIO_MASK_11        0x800 // IO_11
#define GPIO_MASK_IN_GO    0x1000 // IO_GO (etat pin)
#define GPIO_MASK_GO       0x2000 // GO : match lance

// maping
#define GPIO_MASK(a)       (1<<a)

#define IO_COLOR           GPIO_0
#define IO_OMRON1          GPIO_1
#define IO_OMRON2          GPIO_2
#define IO_OMRON3          GPIO_3
//#define IO_OMRON4          GPIO_4
//#define IO_OMRON5          GPIO_5
//#define IO_OMRON6          GPIO_6
//#define IO_LIDAR_LITE_1    GPIO_7
//#define IO_LIDAR_LITE_2    GPIO_8
#define IO_ELEVATOR_SWITCH GPIO_9
#define IO_ELEVATOR_DIR    GPIO_10
#define IO_ELEVATOR_STEP   GPIO_11
#define IO_GO              GPIO_IN_GO

uint32_t gpio_get_state(void);

int gpio_get(Io io);

void gpio_set(Io io);

void gpio_reset(Io io);

#endif
