#ifndef SYSTICK_H
#define SYSTICK_H

//! @file systick.h
//! @brief Time module
//! @author Atlantronic

#include "cpu/cpu.h"
#include <stdint.h>

// doit être dans une interruption
int systick_reconfigure(uint64_t tick) __attribute__(( __warn_unused_result__ ));

int64_t systick_get_time();
int64_t systick_get_time_from_isr();

int64_t systick_get_match_time();

void systick_start_match();

#endif
