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

void control_straight(float dist);

void control_straight_to_wall(float dist);

void control_rotate(float angle);

void control_goto(float x, float y);

void control_free();

int32_t control_get_state();

#endif
