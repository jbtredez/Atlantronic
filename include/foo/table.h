#ifndef TABLE_H
#define TABLE_H

//! @file table.h
//! @brief Table et obstacles fixes
//! @author Atlantronic

#include <stdint.h>
#include "kernel/math/vect2.h"
#include "kernel/math/polyline.h"

#define TABLE_OBJ_SIZE                      9

extern const struct polyline table_obj[TABLE_OBJ_SIZE];

#endif