#ifndef MOTION_DISABLED_STATE_H
#define MOTION_DISABLED_STATE_H

//! @file MotionDisabledState.h
//! @brief Etat MotionDisabled
//! @author Atlantronic

#include "Motion.h"

class MotionDisabledState : public StateMachineState
{
	public:
		MotionDisabledState();
		virtual void entry(void* data);
		virtual void run(void* data);
		virtual unsigned int transition(void* data, unsigned int currentState);
};

#endif
