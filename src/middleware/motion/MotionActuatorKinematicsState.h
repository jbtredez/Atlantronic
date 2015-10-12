#ifndef MOTION_ACTUATOR_KINEMATICS_STATE_H
#define MOTION_ACTUATOR_KINEMATICS_STATE_H

//! @file MotionActuatorKinematicsState.h
//! @brief Etat MotionActuatorKinematics
//! @author Atlantronic

#include "Motion.h"
#include "MotionMoveState.h"

class MotionActuatorKinematicsState : public MotionMoveState
{
	public:
		MotionActuatorKinematicsState();
		virtual void run(void* data);
		void entry(void* data);
};

#endif
