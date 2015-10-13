#ifndef MOTION_MOVE_STATE_H
#define MOTION_MOVE_STATE_H

//! @file MotionMoveState.h
//! @brief Etat MotionMove
//! @author Atlantronic

#include "Motion.h"

class MotionMoveState : public StateMachineState
{
	public:
		MotionMoveState(const char* name,unsigned int stateId);
		virtual void run(void* data){};
		virtual unsigned int transition(void* data);
};

#endif
