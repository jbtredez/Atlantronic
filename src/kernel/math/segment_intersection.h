#ifndef SEGMENT_INTERSECTION_H
#define SEGMENT_INTERSECTION_H

#include "kernel/vect_pos.h"

int segment_intersection(const struct fx_vect2 a, const struct fx_vect2 b, const struct fx_vect2 c, const struct fx_vect2 d, struct fx_vect2* h);

#endif
