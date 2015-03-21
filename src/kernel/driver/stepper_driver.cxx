#include "stepper_driver.h"


StepperDriver::StepperDriver(Io ioStep, Io ioDir)
{
	m_ioStep = ioStep;
	m_ioDir = ioDir;
	m_kinematics.reset();
	m_wanted_pos = 0;
	m_currentStep = 0;
	m_stepFactor = 5;
	m_kinematicsParam.vMax = 100;
	m_kinematicsParam.aMax = 400;
	m_kinematicsParam.dMax = 400;
}

void StepperDriver::setPosition(float pos)
{
	m_wanted_pos = pos;
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
}
