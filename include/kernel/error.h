#ifndef ERROR_H
#define ERROR_H

//! @file error.h
//! @brief Error history
//! @author Atlantronic

#include "gpio.h"
#include "kernel/error_codes.h"

void error_raise(uint16_t error_number);

#endif
