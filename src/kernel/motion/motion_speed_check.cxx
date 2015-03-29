#include <math.h>
#include "motion_speed_check.h"
#include "kernel/log.h"

MotionSpeedCheck::MotionSpeedCheck(float deltaMax, int delayInCycle)
{
	m_deltaMax = deltaMax;
	m_delayInCycle = delayInCycle;
	m_errorCycleCount = 0;
}

enum motion_check_speed MotionSpeedCheck::compute(float cons, float mes)
{
	enum motion_check_speed res = MOTION_SPEED_OK;

	if( fabsf(mes) > fabsf(cons) + m_deltaMax)
	{
		if( m_errorCycleCount < m_delayInCycle )
		{
			m_errorCycleCount++;
		}
		else
		{
			log_format(LOG_ERROR, "over speed %d instead of %d", (int)mes, (int)cons);
			res = MOTION_OVER_SPEED;
		}
	}
	else if( fabsf(cons) > fabsf(mes) + m_deltaMax )
	{
		if( m_errorCycleCount < m_delayInCycle )
		{
			m_errorCycleCount++;
		}
		else
		{
			log_format(LOG_ERROR, "under speed %d instead of %d", (int)mes, (int)cons);
			res = MOTION_UNDER_SPEED;
		}
	}
	else if( fabsf(cons - mes) > m_deltaMax && cons * mes < 0)
	{
		if( m_errorCycleCount < m_delayInCycle )
		{
			m_errorCycleCount++;
		}
		else
		{
			log_format(LOG_ERROR, "wrong speed way %d instead of %d", (int)mes, (int)cons);
			res =  MOTION_WRONG_WAY;
		}
	}
	else
	{
		m_errorCycleCount = 0;
	}

	return res;
}
