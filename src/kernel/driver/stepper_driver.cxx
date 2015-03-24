#include "stepper_driver.h"
#include "kernel/log.h"
#include <stdlib.h>

StepperDriver::StepperDriver(Io ioStep, Io ioDir, float stepFactor, float vmax, float amax, float dmax)
{
	m_ioStep = ioStep;
	m_ioDir = ioDir;
	m_kinematics.reset();
	m_wanted_pos = 0;
	m_currentStep = 0;
	m_stepFactor = stepFactor;
	m_stepFactorInv = 1 / stepFactor;
	m_kinematicsParam.vMax = vmax;
	m_kinematicsParam.aMax = amax;
	m_kinematicsParam.dMax = dmax;
}

void StepperDriver::step(float dt)
{
	m_kinematics.setPosition(m_wanted_pos, 0, m_kinematicsParam, dt);
	int wantedStep = m_stepFactor * m_kinematics.pos;
	if( m_currentStep < wantedStep)
	{
		// on tourne dans le sens positif
		gpio_set(m_ioDir);
		gpio_set(m_ioStep);
		m_currentStep++;
		gpio_reset(m_ioStep);
	}
	else if( m_currentStep > wantedStep)
	{
		// on tourne dans le sens positif
		gpio_reset(m_ioDir);
		gpio_set(m_ioStep);
		m_currentStep--;
		gpio_reset(m_ioStep);
	}

	if( abs(m_currentStep - wantedStep) > 2 )
	{
		log(LOG_ERROR, "stepper driver too slow");
	}
}
