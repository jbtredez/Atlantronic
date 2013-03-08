#ifndef UTF8_H
#define UTF8_H

//! @file serial_number.h
//! @brief Serial number of stm32 (96 bit)
//! @author Atlantronic

#include <stdint.h>

void uint_to_hex_utf8(uint32_t val, uint8_t *pbuf, uint8_t len);

#endif

