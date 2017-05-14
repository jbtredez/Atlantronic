#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

//! @file stepper_driver.h
//! @brief Gestion d'un moteur pas a pas via driver A4988
//! @author Atlantronic

#include <stdint.h>
#include "kernel/driver/io.h"
#include "kernel/control/kinematics.h"

class StepperDriver
{
	public :
		int init(Io ioStep, Io ioDir, float stepFactor, float vmax, float amax, float dmax);

		inline void setPosition(float pos)
		{
			m_wanted_pos = pos;
		}

		inline float getWantedPosition()
		{
			return m_wanted_pos;
		}

		inline void setCurrentPosition(float pos)
		{
			m_kinematics.pos = pos;
			m_currentStep = pos * m_stepFactor;
		}

		inline float getCurrentPosition()
		{
			return m_currentStep * m_stepFactorInv;
		}

		void step(float dt);

		inline void setMaxSpeed(float vMax)
		{
			m_kinematicsParam.vMax = vMax;
		}

	protected:
		Io m_ioStep;
		Io m_ioDir;
		Kinematics m_kinematics;
		KinematicsParameters m_kinematicsParam;
		float m_wanted_pos;
		float m_stepFactor;
		float m_stepFactorInv;
		int m_currentStep;
};

#endif
