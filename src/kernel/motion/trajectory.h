#ifndef TRAJECTORY_H
#define TRAJECTORY_H

//! @file trajectory.h
//! @brief Generation de trajectoire
//! @author Atlantronic

#include "motion.h"
#include "kernel/location/location.h"

enum trajectory_cmd_type
{
	TRAJECTORY_FREE,
	TRAJECTORY_STRAIGHT,
	TRAJECTORY_STRAIGHT_TO_WALL,
	TRAJECTORY_ROTATE,
	TRAJECTORY_ROTATE_TO,
	TRAJECTORY_GOTO_XY,
	TRAJECTORY_GOTO_XYA,
	TRAJECTORY_GOTO_GRAPH,
};

enum avoidance_type
{
	AVOIDANCE_STOP,        //!< arrêt en cas d'obstacle
	AVOIDANCE_GRAPH,       //!< passage par le graph en cas d'obstacle
};

enum trajectory_state
{
	TRAJECTORY_STATE_NONE,
	TRAJECTORY_STATE_MOVE_TO_DEST,
	TRAJECTORY_STATE_MOVING_TO_DEST,
	TRAJECTORY_STATE_MOVE_TO_GRAPH,
	TRAJECTORY_STATE_USING_GRAPH,
	TRAJECTORY_STATE_TARGET_REACHED,
	TRAJECTORY_STATE_TARGET_NOT_REACHED,
	TRAJECTORY_STATE_COLISION,
};

struct trajectory_cmd_arg
{
	uint16_t type;             //!< type de trajectoire
	uint16_t avoidance_type;   //!< type d'évitement
	uint16_t way;              //!< sens
	VectPlan dest;
	float dist;
} __attribute__ (( packed ));

void trajectory_set_kinematics_param(KinematicsParameters linParam, KinematicsParameters angParam);

//!< roue libre
void trajectory_free();

//!< rejoindre le graph
void trajectory_goto_graph();

void trajectory_goto_graph_node(uint32_t node_id, float dist, enum motion_way way, enum avoidance_type avoidance_type);

void trajectory_goto_near_xy(float x, float y, float dist, enum motion_way way, enum avoidance_type avoidance_type);

void trajectory_goto_near(VectPlan dest, float dist, enum motion_way way, enum avoidance_type avoidance_type);

void trajectory_goto(VectPlan dest, enum motion_way way, enum avoidance_type avoidance_type);

void trajectory_rotate(float theta);

void trajectory_rotate_to(float theta);

void trajectory_straight(float dist);

void trajectory_straight_to_wall();

//!< desactivation de l'arrêt sur obstacle détecté par hokuyo
void trajectory_disable_hokuyo();

//!< activation de l'arrêt sur obstacle détecté par hokuyo
void trajectory_enable_hokuyo();

//!< desactivation de l'arrêt sur obstacle statique
void trajectory_enable_static_check();

//!< activation de l'arrêt sur obstacle statique
void trajectory_disable_static_check();

enum trajectory_state trajectory_get_state();

int trajectory_wait(enum trajectory_state wanted_state, uint32_t timeout);

#endif
