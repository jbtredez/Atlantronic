#include "ArmCm3.h"

ArmCm3::ArmCm3(Model *m)
{
	pthread_mutex_init(&io, NULL);
	model = m;
}

ArmCm3::~ArmCm3()
{

}

void ArmCm3::update_hardware(uint64_t vm_clk)
{
	model->update(vm_clk);
}

void ArmCm3::mem_write(uint64_t offset, uint32_t val)
{
	offset += PERIPH_BASE;

	pthread_mutex_lock(&io);

	if( offset >= RCC_BASE && offset < (RCC_BASE + 0x400) )
	{
		RCC.mem_write(offset - RCC_BASE, val);
	}
	else if( offset >= TIM1_BASE && offset < (TIM1_BASE + 0x400) )
	{
		TIM1.mem_write(offset - TIM1_BASE, val);
	}
	else if( offset >= TIM3_BASE && offset < (TIM3_BASE + 0x400) )
	{
		TIM3.mem_write(offset - TIM3_BASE, val);
	}
	else if( offset >= TIM4_BASE && offset < (TIM4_BASE + 0x400) )
	{
		TIM4.mem_write(offset - TIM4_BASE, val);
	}
	else if( offset >= GPIOD_BASE && offset < (GPIOD_BASE + 0x400))
	{
		GPIOD.mem_write(offset - GPIOD_BASE, val);
	}
	else
	{
		meslog(_erreur_, "write non supporté offset %lx, val %x\n", offset, val);
	}

	pthread_mutex_unlock(&io);
}

uint32_t ArmCm3::mem_read(uint64_t offset)
{
	uint32_t rep = 0;

	offset += PERIPH_BASE;

	pthread_mutex_lock(&io);

	if( offset >= RCC_BASE && offset < (RCC_BASE + 0x400) )
	{
		rep = RCC.mem_read(offset - RCC_BASE);
	}
	else if( offset >= TIM1_BASE && offset < (TIM1_BASE + 0x400) )
	{
		rep = TIM1.mem_read(offset - TIM1_BASE);
	}
	else if( offset >= TIM3_BASE && offset < (TIM3_BASE + 0x400) )
	{
		rep = TIM3.mem_read(offset - TIM3_BASE);
	}
	else if( offset >= TIM4_BASE && offset < (TIM4_BASE + 0x400) )
	{
		rep = TIM4.mem_read(offset - TIM4_BASE);
	}
	else if( offset >= GPIOD_BASE && offset < (GPIOD_BASE + 0x400))
	{
		rep = GPIOD.mem_read(offset - GPIOD_BASE);
	}
	else
	{
		meslog(_erreur_, "read non supporté offset %lx\n", offset);
	}

	pthread_mutex_unlock(&io);

	return rep;
}
