#ifndef MOTION_ACTUATOR_KINEMATICS_STATE_H
#define MOTION_ACTUATOR_KINEMATICS_STATE_H

//! @file MotionActuatorKinematicsState.h
//! @brief Etat MotionActuatorKinematics
//! @author Atlantronic

#include "Motion.h"

class MotionActuatorKinematicsState : public StateMachineState
{
	public:
		MotionActuatorKinematicsState();
		virtual void run(void* data);
		virtual unsigned int transition(void* data, unsigned int currentState);
};

#endif
