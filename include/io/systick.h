#include "cpu/cpu.h"
#include <stdint.h>

// doit être dans une interruption
int systick_reconfigure(uint64_t tick) __attribute__(( __warn_unused_result__ ));