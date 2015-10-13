#ifndef MOTION_SPEED_STATE_H
#define MOTION_SPEED_STATE_H

//! @file MotionSpeedState.h
//! @brief Etat MotionSpeed
//! @author Atlantronic

#include "Motion.h"

#include "MotionMoveState.h"
class MotionSpeedState : public MotionMoveState
{
	public:
		MotionSpeedState();
		virtual void entry(void* data);
		virtual void run(void* data);
};

#endif
