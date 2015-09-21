#ifndef MOTION_SPEED_STATE_H
#define MOTION_SPEED_STATE_H

//! @file MotionSpeedState.h
//! @brief Etat MotionSpeed
//! @author Atlantronic

#include "motion.h"

class MotionSpeedState : public StateMachineState
{
	public:
		MotionSpeedState();
		virtual void entry(void* data);
		virtual void run(void* data);
		virtual unsigned int transition(void* data, unsigned int currentState);
};

#endif
