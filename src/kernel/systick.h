#ifndef SYSTICK_H
#define SYSTICK_H

//! @file systick.h
//! @brief Time module
//! @author Atlantronic

#include "kernel/cpu/cpu.h"
#include <stdint.h>

int64_t systick_get_time();
int64_t systick_get_time_from_isr();

int64_t systick_get_match_time();

//!< enregistrement du temps du debut du match (si match non débuté)
void systick_start_match();

//!< enregistrement du temps du debut du match (si match non débuté)
void systick_start_match_from_isr();

#endif
