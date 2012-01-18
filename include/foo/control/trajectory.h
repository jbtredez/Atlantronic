#ifndef TRAJECTORY_H
#define TRAJECTORY_H

//! @file trajectory.h
//! @brief Generation de trajectoire
//! @author Atlantronic

#include <stdint.h>
#include "kernel/vect_pos.h"
#include "kernel/trapeze.h"

#define TRAJECTORY_MAX_INTER_PT          10

enum trajectory_way
{
	TRAJECTORY_ANY_WAY,
	TRAJECTORY_FORWARD,
	TRAJECTORY_BACKWARD
};

struct trajectory
{
	struct vect_pos pt[TRAJECTORY_MAX_INTER_PT]; //!< point de passage
	uint32_t num_pt;                             //!< nombre de points (>= 1)
	uint32_t current;                            //!< id de la destination courante
	enum trajectory_way sens;
	float dist_approx;

	struct vect_pos pos_cons;      //!< position de consigne actuelle
	struct vect_pos dest_cons;     //!< destination de consigne actuelle ( différent de "end" en cas d'evitement, c'est un point de passage)
	struct trapeze trapeze_rot;
	struct trapeze trapeze_av;

	float angle;
	float dist;
};

void trajectory_reset(struct trajectory* t);

void trajectory_add_point(struct trajectory* t, struct vect_pos* pt);

void trajectory_straight(struct trajectory* t, float dist);

void trajectory_rotate(struct trajectory* t, float angle);

void trajectory_rotate_to(struct trajectory* t, float angle);

void trajectory_goto(struct trajectory* t, float x, float y, float dist, enum trajectory_way sens);

int trajectory_compute(struct trajectory* t, struct vect_pos* pos_mes);

#endif
