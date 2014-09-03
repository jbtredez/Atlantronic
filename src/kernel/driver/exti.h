#ifndef EXTI_H
#define EXTI_H

#include <stdint.h>

typedef long (*ExtiIsr)(void);

typedef enum
{
	EXTI_PA = 0,
	EXTI_PB,
	EXTI_PC,
	EXTI_PD,
	EXTI_PE,
	EXTI_PF,
	EXTI_PG,
} ExtiPort;

#define EXTI_TYPE_DOWN       0x01
#define EXTI_TYPE_UP         0x02

int exti_register(ExtiPort port, uint8_t pin, uint8_t type, ExtiIsr isr);

#endif
