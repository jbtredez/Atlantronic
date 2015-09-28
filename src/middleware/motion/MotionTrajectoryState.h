#ifndef MOTION_TRAJECTORY_STATE_H
#define MOTION_TRAJECTORY_STATE_H

//! @file MotionTrajectoryState.h
//! @brief Etat MotionTrajectory
//! @author Atlantronic

#include "Motion.h"

class MotionTrajectoryState : public StateMachineState
{
	public:
		MotionTrajectoryState();
		virtual void entry(void* data);
		virtual void run(void* data);
		virtual unsigned int transition(void* data, unsigned int currentState);
};

#endif
