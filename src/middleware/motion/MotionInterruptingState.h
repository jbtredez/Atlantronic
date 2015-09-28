#ifndef MOTION_INTERRUPTING_STATE_H
#define MOTION_INTERRUPTING_STATE_H

//! @file MotionInterruptingState.h
//! @brief Etat MotionInterrupting
//! @author Atlantronic

#include "Motion.h"

class MotionInterruptingState : public StateMachineState
{
	public:
		MotionInterruptingState();
		virtual void run(void* data);
		virtual unsigned int transition(void* data, unsigned int currentState);
};

#endif
