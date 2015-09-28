#ifndef MOTION_ENABLED_STATE_H
#define MOTION_ENABLED_STATE_H

//! @file MotionEnabledState.h
//! @brief Etat MotionEnabled
//! @author Atlantronic

#include "Motion.h"

class MotionEnabledState : public StateMachineState
{
	public:
		MotionEnabledState();
		virtual void entry(void* data);
		virtual void run(void* data);
		virtual unsigned int transition(void* data, unsigned int currentState);
};

#endif
