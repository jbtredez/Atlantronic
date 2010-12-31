#ifndef CPU_H
#define CPU_H

#ifdef __ARM_CM3__
	#define STM32F10X_CL
	#include "cpu/stm32f10x.h"
#endif

#ifdef __GCC_POSIX__
	#define STM32F10X_CL
	#include "cpu/stm32f10x_simu.h"
#endif

#endif
