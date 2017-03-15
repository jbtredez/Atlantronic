#ifndef TABLE_H
#define TABLE_H

//! @file table.h
//! @brief Table et obstacles fixes
//! @author Atlantronic

#include <stdint.h>
#include "kernel/math/polyline.h"

#define TABLE_OBJ_SIZE                      10

extern struct polyline table_obj[TABLE_OBJ_SIZE];

void setTableColor(int color);

#endif
