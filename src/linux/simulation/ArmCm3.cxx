#include "ArmCm3.h"

ArmCm3::ArmCm3(Model *m) :
	CpuEmu(),
	USART3(this, USART3_IRQn)
{
	pthread_mutex_init(&io, NULL); //////// TODO
	model = m;

	connect_io(RCC_BASE, 0x400, &rcc);
	connect_io(GPIOA_BASE, 0x400, &gpioA);
	connect_io(GPIOB_BASE, 0x400, &gpioB);
	connect_io(GPIOC_BASE, 0x400, &gpioC);
	connect_io(GPIOD_BASE, 0x400, &gpioD);
	connect_io(GPIOE_BASE, 0x400, &gpioE);
	connect_io(GPIOF_BASE, 0x400, &gpioF);
	connect_io(GPIOG_BASE, 0x400, &gpioG);
	connect_io(TIM1_BASE, 0x400, &tim1);
	connect_io(TIM2_BASE, 0x400, &tim2);
	connect_io(TIM3_BASE, 0x400, &tim3);
	connect_io(TIM4_BASE, 0x400, &tim4);
	connect_io(TIM5_BASE, 0x400, &tim5);
	connect_io(TIM6_BASE, 0x400, &tim6);
	connect_io(TIM7_BASE, 0x400, &tim7);
	connect_io(ADC1_BASE, 0x400, &adc1);
	connect_io(ADC2_BASE, 0x400, &adc2);
	connect_io(CAN1_BASE, 0x400, &can1);
}

ArmCm3::~ArmCm3()
{

}

void ArmCm3::update_hardware(uint64_t vm_clk)
{
	model->update(vm_clk);
}
