#ifndef ERROR_H
#define ERROR_H

//! @file error.h
//! @brief Error history
//! @author Atlantronic

#include "gpio.h"
#include "kernel/error_codes.h"

#define ERROR_CLEAR            0x00
#define ERROR_ACTIVE           0x01

void _error(enum fault id, unsigned char new_state, const char* func, int line);

long _error_from_isr(enum fault id, unsigned char new_state, const char* func, int line);

void _error_check_update(enum fault id, uint32_t err, const char* func, int line);

#define error_check_update(id, err) _error_check_update(id, err, __FUNCTION__, __LINE__)

#define error(id, new_state) _error(id, new_state, __FUNCTION__, __LINE__)

#define error_from_isr(id, new_state) _error_from_isr(id, new_state, __FUNCTION__, __LINE__)

#endif
