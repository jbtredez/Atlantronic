#ifndef SERIAL_NUMBER_H
#define SERIAL_NUMBER_H

//! @file serial_number.h
//! @brief Serial number of stm32 (96 bit)
//! @author Atlantronic

#define SERIAL_NUMBER_0     *((volatile uint32_t*)0x1FFFF7E8)
#define SERIAL_NUMBER_1     *((volatile uint32_t*)0x1FFFF7EC)
#define SERIAL_NUMBER_2     *((volatile uint32_t*)0x1FFFF7F0)


#endif

