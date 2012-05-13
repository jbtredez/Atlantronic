#ifndef TRAJECTORY_H
#define TRAJECTORY_H

//! @file trajectory.h
//! @brief Generation de trajectoire
//! @author Atlantronic

#include "control/control.h"
#include "location/location.h"

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

enum trajectory_avoidance_type
{
	TRAJECTORY_AVOIDANCE_STOP,        //!< arrêt en cas d'obstacle
	TRAJECTORY_AVOIDANCE_GRAPH,       //!< passage par le graph en cas d'obstacle
};

enum trajectory_state
{
	TRAJECTORY_STATE_NONE,
	TRAJECTORY_STATE_MOVING_TO_DEST,
	TRAJECTORY_STATE_MOVING_TO_GRAPH,
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
	int32_t x;
	int32_t y;
	int32_t alpha;
	int32_t dist;
} __attribute__ (( packed ));

//!< roue libre
void trajectory_free();

//!< rejoindre le graph
void trajectory_goto_graph();

void trajectory_goto_graph_node(uint32_t node_id, int32_t dist, enum trajectory_way way, enum trajectory_avoidance_type avoidance_type);

void trajectory_goto_near_xy(int32_t x, int32_t y, int32_t dist, enum trajectory_way way, enum trajectory_avoidance_type avoidance_type);

void trajectory_goto_near(int32_t x, int32_t y, int32_t alpha, int32_t dist, enum trajectory_way way, enum trajectory_avoidance_type avoidance_type);

void trajectory_rotate(int32_t angle);

void trajectory_rotate_to(int32_t angle);

void trajectory_straight(int32_t dist);

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

#endif