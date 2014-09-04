#ifndef LED_H
#define LED_H

#define LED_CPU_RED      0x00004000
#define LED_CPU_BLUE     0x00008000
#define LED_EXT_BLUE     0x80000000
#define LED_EXT_GREEN    0x20000000
#define LED_EXT_ORANGE1  0x00100000
#define LED_EXT_ORANGE2  0x00040000
#define LED_EXT_RED      0x01000000

void setLed(uint32_t mask);

#endif
