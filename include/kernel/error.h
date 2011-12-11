#ifndef ERROR_H
#define ERROR_H

//! @file error.h
//! @brief Error history
//! @author Atlantronic

#include <stdint.h>
#include "kernel/error_codes.h"

#define ERROR_CLEAR            0x00
#define ERROR_ACTIVE           0x01

struct error_status
{
	unsigned char state;
	uint64_t time;
} __attribute__((packed));

void error(enum fault id, unsigned char new_state);

long error_from_isr(enum fault id, unsigned char new_state);

void error_check_update(enum fault id, uint32_t err);

#endif
