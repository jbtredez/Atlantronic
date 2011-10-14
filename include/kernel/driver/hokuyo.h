//! @file hokuyo.h
//! @brief Hokuyo module
//! @author Atlantronic

#include <stdint.h>

uint32_t hokuyo_init();
uint32_t hokuyo_scan(float x, float y, float alpha);
uint32_t hokuyo_decode_distance(uint16_t* distance, int size);