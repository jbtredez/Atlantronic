#ifndef ARM_RCC_H
#define ARM_RCC_H

//! @file ArmRcc.h
//! @brief RCC
//! @author Jean-Baptiste Trédez

#include "ArmMem.h"
#include "log.h"

class ArmRcc : public ArmMem<RCC_TypeDef>
{
public:
	ArmRcc();
	~ArmRcc();

	void mem_write(uint64_t offset, uint32_t val);

protected:
	void update(uint64_t offset);
};

#endif
