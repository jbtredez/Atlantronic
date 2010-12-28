#ifndef ARM_RCC_H
#define ARM_RCC_H

//! @file ArmRcc.h
//! @brief RCC
//! @author Jean-Baptiste Trédez

#include "ArmMem.h"

// TODO : voir / envoyer à qemu le system_clock_scale

class ArmRcc : public ArmMem<RCC_TypeDef>
{
public:
	ArmRcc();
	~ArmRcc();

private:
	int system_clock_scale;
	void update();
};

#endif