#ifndef CONTROL_H
#define CONTROL_H

//! @file control.h
//! @brief Asservissement
//! @author Atlantronic

#include <stdint.h>
#include "kernel/vect_pos.h"

enum control_state
{
	CONTROL_READY_ASSER,          //!< no trajectory ongoing
	CONTROL_READY_FREE,
	CONTROL_ROTATE,               //!< rotate
	CONTROL_STRAIGHT,             //!< go straight
	CONTROL_STRAIGHT_TO_WALL,     //!< on recule dans un mur pour recaler. On va tout droit au debut puis on desactive l'asservissement en rotation dés qu'un coté touche le mur
	CONTROL_GOTO,                 //!< goto
	CONTROL_ARC,                  //!< arc
	CONTROL_END,                  //!< end : halted forever
};

enum control_way
{
	CONTROL_ANY_WAY,
	CONTROL_FORWARD,
	CONTROL_BACKWARD
};

struct control_usb_data
{
	int32_t control_state;
	float control_dest_x;
	float control_dest_y;
	float control_dest_alpha;
	float control_cons_x;
	float control_cons_y;
	float control_cons_alpha;
	float control_pos_x;
	float control_pos_y;
	float control_pos_alpha;
	float control_v_dist_cons;
	float control_v_rot_cons;
};

void control_straight(float dist);

void control_straight_to_wall(float dist);

void control_rotate(float angle);

void control_rotate_to(float alpha);

void control_goto_near(float x, float y, float dist, enum control_way sens);

void control_free();

void control_set_use_us(uint8_t use_us_mask);

int32_t control_get_state();

#endif
