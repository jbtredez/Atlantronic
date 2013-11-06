//! @file vect_pos.c
//! @brief Changements de reperes
//! @author Atlantronic

#include "kernel/vect_pos.h"
#include "kernel/math/fx_math.h"

//! changement de repere du repère local au repere absolu
//! origin : origine du repère local dans le repère absolu
void vect2_loc_to_abs(const struct fx_vect_pos *origin, const struct fx_vect2 *pos_in, struct fx_vect2 *pos_out)
{
	pos_out->x = origin->x + (((int64_t)origin->ca * (int64_t)pos_in->x - (int64_t)origin->sa * (int64_t)pos_in->y) >> 30);
	pos_out->y = origin->y + (((int64_t)origin->sa * (int64_t)pos_in->x + (int64_t)origin->ca * (int64_t)pos_in->y) >> 30);
}

//! changement de repere du repère local au repere absolu
//! origin : origine du repère local dans le repère absolu
void pos_loc_to_abs(const struct fx_vect_pos *origin, const struct fx_vect_pos *pos_in, struct fx_vect_pos *pos_out)
{
	pos_out->x = origin->x + (((int64_t)origin->ca * (int64_t)pos_in->x - (int64_t)origin->sa * (int64_t)pos_in->y) >> 30);
	pos_out->y = origin->y + (((int64_t)origin->sa * (int64_t)pos_in->x + (int64_t)origin->ca * (int64_t)pos_in->y) >> 30);
	pos_out->alpha = pos_in->alpha + origin->alpha;
	pos_out->ca = fx_cos(pos_out->alpha);
	pos_out->sa = fx_sin(pos_out->alpha);
}

//! changement de repere du repère absolu au repere local
//! origin : origine du repère local dans le repère absolu
void pos_abs_to_loc(const struct fx_vect_pos *origin, const struct fx_vect_pos *pos_in, struct fx_vect_pos *pos_out)
{
	int32_t x = pos_in->x - origin->x;
	int32_t y = pos_in->y - origin->y;
	pos_out->x = ((int64_t)origin->ca * (int64_t)x + (int64_t)origin->sa * (int64_t)y) >> 30;
	pos_out->y = (- (int64_t)origin->sa * (int64_t)x + (int64_t)origin->ca * (int64_t)y ) >> 30;
	pos_out->alpha = pos_in->alpha - origin->alpha;
	pos_out->ca = fx_cos(pos_out->alpha);
	pos_out->sa = fx_sin(pos_out->alpha);
}

//! changement de repere du repère absolu au repere local
//! origin : origine du repère local dans le repère absolu
void vect2_abs_to_loc(const struct fx_vect_pos *origin, const struct fx_vect2 *pos_in, struct fx_vect2 *pos_out)
{
	int32_t x = pos_in->x - origin->x;
	int32_t y = pos_in->y - origin->y;
	pos_out->x = (  (int64_t)origin->ca * (int64_t)x + (int64_t)origin->sa * (int64_t)y) >> 30;
	pos_out->y = (- (int64_t)origin->sa * (int64_t)x + (int64_t)origin->ca * (int64_t)y) >> 30;
}
