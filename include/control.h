#ifndef CONTROL_H
#define CONTROL_H

//! @file control.h
//! @brief Asservissement
//! @author Jean-Baptiste Trédez

#include <stdint.h>
#include "vect_pos.h"

enum control_state
{
	READY,          //!< no trajectory ongoing
	ROTATE,         //!< rotate
	STRAIGHT,       //!< go straight
	GOTO,           //!< goto
	ARC,            //!< arc
};

void control_straight(float dist);

void control_rotate(float angle);

void control_goto(float x, float y);

#endif
