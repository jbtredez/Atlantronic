#include "StepperDriver.h"
#include "kernel/log.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/module.h"
#include <stdlib.h>

#define STEPPER_DRIVER_STACK_SIZE       200
#define STEPPER_DRIVER_MAX                8

static StepperDriver* stepperDriver[STEPPER_DRIVER_MAX];
static int stepper_driver_count = 0;

static void stepper_driver_task(void* arg);

static int stepper_driver_module_init()
{
	portBASE_TYPE err = xTaskCreate(stepper_driver_task, "stepper_driver", STEPPER_DRIVER_STACK_SIZE, NULL, PRIORITY_TASK_STEPPER_DRIVER, NULL);

	if(err != pdPASS)
	{
		return ERR_INIT_STEPPER_DRIVER;
	}

	return 0;
}

module_init(stepper_driver_module_init, INIT_STEPPER_DRIVER);

static void stepper_driver_task(void* /*arg*/)
{
	uint32_t wake_time = 0;

	while(1)
	{
		for(int i = 0; i < stepper_driver_count; i++)
		{
			stepperDriver[i]->step(0.001);
		}

		vTaskDelayUntil(&wake_time, 1);
	}
}

int StepperDriver::init(Io ioStep, Io ioDir, float stepFactor, float vmax, float amax, float dmax)
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

	if( stepper_driver_count >= STEPPER_DRIVER_MAX )
	{
		log(LOG_INFO, "too many stepper driver registered");
		return -1;
	}

	stepperDriver[stepper_driver_count] = this;
	stepper_driver_count++;
	log_format(LOG_INFO, "stepper driver %d registered", stepper_driver_count-1);

	return 0;
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
