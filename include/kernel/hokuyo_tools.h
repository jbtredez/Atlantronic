//! @file hokuyo_tools.h
//! @brief Hokuyo tools
//! @author Jean-Baptiste Trédez

#ifndef HOKUYO_TOOLS_H
#define HOKUYO_TOOLS_H

#include <stdint.h>

uint16_t hokuyo_tools_decode16(const unsigned char* data);

int hokuyo_tools_decode_buffer(const unsigned char* buffer, unsigned int buffer_size, uint16_t* distance, unsigned int distance_size);

#endif
