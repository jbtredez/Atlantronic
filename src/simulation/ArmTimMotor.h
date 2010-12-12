#ifndef ARM_TIM_MOTOR_H
#define ARM_TIM_MOTOR_H

//! @file ArmTim.h
//! @brief TIM
//! @author Jean-Baptiste Trédez

#include "ArmMem.h"
#include "Motor.h"

class ArmTimMotor : public ArmMem<TIM_TypeDef>
{
public:
	ArmTimMotor();
	~ArmTimMotor();

	void update();
	Motor motor[4];
};

#endif
