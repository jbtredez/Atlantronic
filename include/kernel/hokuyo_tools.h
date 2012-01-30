#ifndef HOKUYO_TOOLS_H
#define HOKUYO_TOOLS_H

//! @file hokuyo_tools.h
//! @brief Hokuyo tools
//! @author Atlantronic

#include <stdint.h>
#include "kernel/driver/hokuyo.h"
#include "kernel/vect_pos.h"

struct hokuyo_object
{
	uint16_t start;
	uint16_t size;
};

void hokuyo_precompute_angle(struct hokuyo_scan* scan, struct fx_vect2 *csangle);

void hokuyo_compute_xy(struct hokuyo_scan* scan, struct fx_vect2 *pos, struct fx_vect2 *csangle);

int hokuyo_find_objects(uint16_t* distance, unsigned int size, struct hokuyo_object* obj, unsigned int obj_size);

int hokuyo_object_is_pawn(uint16_t* distance, struct hokuyo_object* obj, struct vect_pos *pawn_pos);

#endif
