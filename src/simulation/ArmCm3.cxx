#include "ArmCm3.h"

ArmCm3::ArmCm3(Model *m) :
	CpuEmu(),
	USART3(this, USART3_IRQn)
{
	pthread_mutex_init(&io, NULL);
	model = m;

	connect_io(RCC_BASE, 0x400, &rcc);
}

ArmCm3::~ArmCm3()
{

}

void ArmCm3::update_hardware(uint64_t vm_clk)
{
	model->update(vm_clk);
}


/*
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
	else if( offset >= GPIOA_BASE && offset < (GPIOA_BASE + 0x400))
	{
		GPIOA.mem_write(offset - GPIOA_BASE, val);
	}
	else if( offset >= GPIOB_BASE && offset < (GPIOB_BASE + 0x400))
	{
		GPIOB.mem_write(offset - GPIOB_BASE, val);
	}
	else if( offset >= GPIOC_BASE && offset < (GPIOC_BASE + 0x400))
	{
		GPIOC.mem_write(offset - GPIOC_BASE, val);
	}
	else if( offset >= GPIOD_BASE && offset < (GPIOD_BASE + 0x400))
	{
		GPIOD.mem_write(offset - GPIOD_BASE, val);
	}
	else if( offset >= GPIOE_BASE && offset < (GPIOE_BASE + 0x400))
	{
		GPIOE.mem_write(offset - GPIOE_BASE, val);
	}
	else if( offset >= GPIOF_BASE && offset < (GPIOF_BASE + 0x400))
	{
		GPIOF.mem_write(offset - GPIOF_BASE, val);
	}
	else if( offset >= GPIOG_BASE && offset < (GPIOG_BASE + 0x400))
	{
		GPIOG.mem_write(offset - GPIOG_BASE, val);
	}
	else if( offset >= USART3_BASE && offset < (USART3_BASE + 0x400))
	{
		USART3.mem_write(offset - USART3_BASE, val);
	}
	else
	{
		meslog(_erreur_, "write non supporté offset %#lx, val %#x", offset, val);
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
	else if( offset >= GPIOA_BASE && offset < (GPIOA_BASE + 0x400))
	{
		rep = GPIOA.mem_read(offset - GPIOA_BASE);
	}
	else if( offset >= GPIOB_BASE && offset < (GPIOB_BASE + 0x400))
	{
		rep = GPIOB.mem_read(offset - GPIOB_BASE);
	}
	else if( offset >= GPIOC_BASE && offset < (GPIOC_BASE + 0x400))
	{
		rep = GPIOC.mem_read(offset - GPIOC_BASE);
	}
	else if( offset >= GPIOD_BASE && offset < (GPIOD_BASE + 0x400))
	{
		rep = GPIOD.mem_read(offset - GPIOD_BASE);
	}
	else if( offset >= GPIOE_BASE && offset < (GPIOE_BASE + 0x400))
	{
		rep = GPIOE.mem_read(offset - GPIOE_BASE);
	}
	else if( offset >= GPIOF_BASE && offset < (GPIOF_BASE + 0x400))
	{
		rep = GPIOF.mem_read(offset - GPIOF_BASE);
	}
	else if( offset >= GPIOG_BASE && offset < (GPIOG_BASE + 0x400))
	{
		rep = GPIOG.mem_read(offset - GPIOG_BASE);
	}
	else if( offset >= USART3_BASE && offset < (USART3_BASE + 0x400))
	{
		rep = USART3.mem_read(offset - USART3_BASE);
	}
	else
	{
		meslog(_erreur_, "read non supporté offset %#lx", offset);
	}

	pthread_mutex_unlock(&io);

	return rep;
}
*/