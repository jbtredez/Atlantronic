#ifndef ARM_RCC_H
#define ARM_RCC_H

//! @file ArmRcc.h
//! @brief RCC
//! @author Jean-Baptiste Trédez

#include "ArmMem.h"
#include "log.h"

// TODO : voir / envoyer à qemu le system_clock_scale

class ArmRcc : public ArmMem<RCC_TypeDef>
{
public:
	ArmRcc();
	~ArmRcc();

private:
	void update(uint64_t offset);
};

#endif
