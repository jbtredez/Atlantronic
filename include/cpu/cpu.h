#ifndef CPU_H
#define CPU_H

#define STM32F10X_CL
#include "cpu/stm32f10x.h"

#ifndef LINUX
#include "asm/asm_base_func.h"
#endif

#ifdef LINUX
	// accès mémoire directs non valides
	#undef InterruptType
	#undef SCB
	#undef SysTick
	#undef NVIC
	#undef ITM
	#undef CoreDebug

	#undef TIM2
	#undef TIM3
	#undef TIM4
	#undef TIM5
	#undef TIM6
	#undef TIM7
	#undef RTC
	#undef WWDG
	#undef IWDG
	#undef SPI2
	#undef SPI3
	#undef USART2
	#undef USART3
	#undef UART4
	#undef UART5
	#undef I2C1
	#undef I2C2
	#undef CAN1
	#undef CAN2
	#undef BKP
	#undef PWR
	#undef DAC
	#undef AFIO
	#undef EXTI
	#undef GPIOA
	#undef GPIOB
	#undef GPIOC
	#undef GPIOD
	#undef GPIOE
	#undef GPIOF
	#undef GPIOG
	#undef ADC1
	#undef ADC2
	#undef TIM1
	#undef SPI1
	#undef TIM8
	#undef USART1
	#undef ADC3
	#undef SDIO
	#undef DMA1
	#undef DMA2
	#undef DMA1_Channel1
	#undef DMA1_Channel2
	#undef DMA1_Channel3
	#undef DMA1_Channel4
	#undef DMA1_Channel5
	#undef DMA1_Channel6
	#undef DMA1_Channel7
	#undef DMA2_Channel1
	#undef DMA2_Channel2
	#undef DMA2_Channel3
	#undef DMA2_Channel4
	#undef DMA2_Channel5
	#undef RCC
	#undef CRC
	#undef FLASH
	#undef OB
	#undef ETH
	#undef FSMC_Bank1
	#undef FSMC_Bank1E
	#undef FSMC_Bank2
	#undef FSMC_Bank3
	#undef FSMC_Bank4
	#undef DBGMCU
#endif

#endif
