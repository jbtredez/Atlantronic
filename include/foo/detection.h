#ifndef DETECTION_H
#define DETECTION_H

//! @file detection.h
//! @brief Detection
//! @author Atlantronic

#include <stdint.h>
#include "kernel/vect_pos.h"

int detection_get_close_pawn(struct vect_pos *best_pawn);

#endif

