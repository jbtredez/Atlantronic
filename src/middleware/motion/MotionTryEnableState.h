#ifndef MOTION_TRY_ENABLE_STATE_H
#define MOTION_TRY_ENABLE_STATE_H

//! @file MotionTryEnableState.h
//! @brief Etat MotionTryEnable
//! @author Atlantronic

#include "motion.h"

class MotionTryEnableState : public StateMachineState
{
	public:
		MotionTryEnableState();
		virtual void run(void* data);
		virtual unsigned int transition(void* data, unsigned int currentState);
};

#endif
