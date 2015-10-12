#ifndef MOTION_INTERRUPTING_STATE_H
#define MOTION_INTERRUPTING_STATE_H

//! @file MotionInterruptingState.h
//! @brief Etat MotionInterrupting
//! @author Atlantronic

#include "Motion.h"

#include "MotionMoveState.h"
class MotionInterruptingState : public MotionMoveState
{
	public:
		MotionInterruptingState();
		virtual void run(void* data);
		virtual unsigned int transition(void* data);
};

#endif
