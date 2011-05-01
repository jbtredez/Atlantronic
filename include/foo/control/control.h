#ifndef CONTROL_H
#define CONTROL_H

//! @file control.h
//! @brief Asservissement
//! @author Jean-Baptiste Trédez

#include <stdint.h>
#include "kernel/vect_pos.h"

enum control_state
{
	CONTROL_READY_ASSERT,          //!< no trajectory ongoing
	CONTROL_READY_FREE,
	CONTROL_ROTATE,         //!< rotate
	CONTROL_STRAIGHT,       //!< go straight
	CONTROL_GOTO,           //!< goto
	CONTROL_ARC,            //!< arc
	CONTROL_END,    //!< end : halted forever
};

void control_straight(float dist);

void control_rotate(float angle);

void control_goto(float x, float y);

void control_free();

int32_t control_get_state();

#endif
