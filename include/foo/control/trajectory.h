#ifndef TRAJECTORY_H
#define TRAJECTORY_H

//! @file trajectory.h
//! @brief Generation de trajectoire
//! @author Atlantronic

#include <stdint.h>
#include "kernel/vect_pos.h"
#include "kernel/trapeze.h"

struct trajectory
{
	struct vect_pos pos_cons;
	struct vect_pos dest_cons;
	struct trapeze trapeze_rot;
	struct trapeze trapeze_av;

	float angle;
	float dist;
};

enum trajectory_way
{
	TRAJECTORY_ANY_WAY,
	TRAJECTORY_FORWARD,
	TRAJECTORY_BACKWARD
};

void trajectory_init_straight(struct trajectory* t, float dist);

void trajectory_init_rotate(struct trajectory* t, float angle);

void trajectory_init_rotate_to(struct trajectory* t, float angle);

void trajectory_init_goto(struct trajectory* t, float x, float y, float dist, enum trajectory_way sens);

int trajectory_compute(struct trajectory* t, struct vect_pos* pos_mes);

#endif
