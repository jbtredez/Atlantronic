#ifndef HOKUYO_TOOLS_H
#define HOKUYO_TOOLS_H

//! @file hokuyo_tools.h
//! @brief Hokuyo tools
//! @author Atlantronic

#include <stdint.h>
#include "kernel/driver/hokuyo.h"
#include "kernel/vect_pos.h"
#include "kernel/math/polyline.h"

void hokuyo_compute_xy(struct hokuyo_scan* scan, struct vect2 *pos);

int hokuyo_find_objects(uint16_t* distance, struct vect2* hokuyo_pos, unsigned int size, struct polyline* obj, unsigned int obj_size);

#endif
