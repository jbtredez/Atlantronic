#ifndef ARM_MEM_H
#define ARM_MEM_H

//! @file ArmMem.h
//! @brief MEM
//! @author Jean-Baptiste Trédez

#include <stdint.h>
#include "cpu/cpu.h"

#include "log.h"
#include <math.h>
#include <stdio.h>

template<typename MEM_TypeDef> class ArmMem
{
public:
	ArmMem();
	~ArmMem();

	virtual void mem_write(uint64_t offset, uint32_t val);
	virtual uint32_t mem_read(uint64_t offset);

protected:
	MEM_TypeDef MEM;
};

template<class MEM_TypeDef>
ArmMem<MEM_TypeDef>::ArmMem()
{
	memset(&MEM, 0x00, sizeof(MEM));
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
		meslog(_erreur_, "MEM - write - non supporté - offset %#lx\n", offset);
		return;
	}
	else
	{
		*((uint32_t*)(((unsigned char*)(&MEM)) + offset)) = val;
	}
}

template<class MEM_TypeDef>
uint32_t ArmMem<MEM_TypeDef>::mem_read(uint64_t offset)
{
	uint32_t rep = 0;
	if(offset > sizeof(MEM_TypeDef) - sizeof(uint32_t))
	{
		meslog(_erreur_, "MEM - read - non supporté - offset %#lx\n", offset);
	}
	else
	{
		rep = *((uint32_t*)(((unsigned char*)(&MEM)) + offset));
	}

	return rep;
}

#endif
