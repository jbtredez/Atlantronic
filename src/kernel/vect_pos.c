//! @file vect_pos.c
//! @brief Changements de reperes
//! @author Atlantronic

#include "kernel/vect_pos.h"
#include <math.h>

//! changement de repere du repère robot au repere table en fonction de la position du robot
void fx_vect2_robot_to_table(struct fx_vect_pos *pos_robot, struct fx_vect2 *pos_in, struct fx_vect2 *pos_out)
{
	pos_out->x = pos_robot->x + (((int64_t)pos_robot->ca * (int64_t)pos_in->x - (int64_t)pos_robot->sa * (int64_t)pos_in->y) >> 30);
	pos_out->y = pos_robot->y + (((int64_t)pos_robot->sa * (int64_t)pos_in->x + (int64_t)pos_robot->ca * (int64_t)pos_in->y) >> 30);
}

//! changement de repere du repère robot au repere table en fonction de la position du robot
void pos_robot_to_table(struct fx_vect_pos *pos_robot, struct fx_vect_pos *pos_in, struct fx_vect_pos *pos_out)
{
	pos_out->x = pos_robot->x + (((int64_t)pos_robot->ca * (int64_t)pos_in->x - (int64_t)pos_robot->sa * (int64_t)pos_in->y) >> 30);
	pos_out->y = pos_robot->y + (((int64_t)pos_robot->sa * (int64_t)pos_in->x + (int64_t)pos_robot->ca * (int64_t)pos_in->y) >> 30);
	pos_out->alpha = pos_in->alpha + pos_robot->alpha;
	pos_out->ca = fx_cos(pos_out->alpha);
	pos_out->sa = fx_sin(pos_out->alpha);
}

//! changement de repere du repère robot au repere table en fonction de la position du robot
void pos_table_to_robot(struct vect_pos *pos_robot, struct vect_pos *pos_in, struct vect_pos *pos_out)
{
	float x = pos_in->x - pos_robot->x;
	float y = pos_in->y - pos_robot->y;
	pos_out->x = pos_robot->ca * x + pos_robot->sa * y;
	pos_out->y = - pos_robot->sa * x + pos_robot->ca * y;
	pos_out->alpha = pos_in->alpha - pos_robot->alpha;
	pos_out->ca = cosf(pos_out->alpha);
	pos_out->sa = sinf(pos_out->alpha);
}

//! changement de repere du repère robot au repere table en fonction de la position du robot
void fx_vect2_table_to_robot(struct fx_vect_pos *pos_robot, struct fx_vect2 *pos_in, struct fx_vect2 *pos_out)
{
	int32_t x = pos_in->x - pos_robot->x;
	int32_t y = pos_in->y - pos_robot->y;
	pos_out->x = (  (int64_t)pos_robot->ca * (int64_t)x + (int64_t)pos_robot->sa * (int64_t)y) >> 30;
	pos_out->y = (- (int64_t)pos_robot->sa * (int64_t)x + (int64_t)pos_robot->ca * (int64_t)y) >> 30;
}

float norm2_square(struct vect_pos *pos)
{
	return pos->x * pos->x + pos->y * pos->y;
}

float distance_square(struct vect_pos *pos1, struct vect_pos *pos2)
{
	float dx = pos2->x - pos1->x;
	float dy = pos2->y - pos1->y;
	return dx * dx + dy * dy;
}
