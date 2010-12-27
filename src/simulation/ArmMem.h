#ifndef ARM_MEM_H
#define ARM_MEM_H

//! @file ArmMem.h
//! @brief MEM
//! @author Jean-Baptiste Trédez

#include <stdint.h>
#define STM32F10X_CL
#include "stm32f10x.h"

#include "log.h"
#include <math.h>
#include <stdio.h>

template<typename MEM_TypeDef> class ArmMem
{
public:
	ArmMem();
	~ArmMem();

	void mem_write(uint64_t offset, uint32_t val);
	uint32_t mem_read(uint64_t offset);

protected:
	virtual void update() = 0;
	MEM_TypeDef MEM;
};

template<class MEM_TypeDef>
ArmMem<MEM_TypeDef>::ArmMem()
{

}

template<class MEM_TypeDef>
ArmMem<MEM_TypeDef>::~ArmMem()
{

}

template<class MEM_TypeDef>
void ArmMem<MEM_TypeDef>::mem_write(uint64_t offset, uint32_t val)
{
	if(offset > sizeof(MEM_TypeDef) - sizeof(uint32_t))
	{
		meslog(_erreur_, "MEM - write - non supporté - offset %li\n", offset);
		return;
	}
	else
	{
		*((uint32_t*)(((unsigned char*)(&MEM)) + offset)) = val;
		update();
	}
}

template<class MEM_TypeDef>
uint32_t ArmMem<MEM_TypeDef>::mem_read(uint64_t offset)
{
	uint32_t rep = 0;
	if(offset > sizeof(MEM_TypeDef) - sizeof(uint32_t))
	{
		meslog(_erreur_, "MEM - read - non supporté - offset %li\n", offset);
	}
	else
	{
		rep = *((uint32_t*)(((unsigned char*)(&MEM)) + offset));
	}

	return rep;
}

#endif
