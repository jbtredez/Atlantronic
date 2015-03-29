#ifndef MOTION_SPEED_CHECK_H
#define MOTION_SPEED_CHECK_H

//! @file motion_speed_check.h

enum motion_check_speed
{
	MOTION_SPEED_OK,
	MOTION_OVER_SPEED,
	MOTION_UNDER_SPEED,
	MOTION_WRONG_WAY,
};

class MotionSpeedCheck
{
	public:
		MotionSpeedCheck(float deltaMax, int delayInCycle);

		enum motion_check_speed compute(float cons, float mes);

	protected:
		float m_deltaMax;
		int m_delayInCycle;
		int m_errorCycleCount;
};

#endif