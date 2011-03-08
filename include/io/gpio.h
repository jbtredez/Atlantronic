#ifndef GPIO_H
#define GPIO_H

//! @file gpio.h
//! @brief Gpio mapping
//! @author Jean-Baptiste Trédez

#ifdef __ARM_CM3__
#include "arch/arm_cm3/gpio.h"
#endif

#ifdef __GCC_POSIX__
#include "arch/gcc_posix/gpio.h"
#endif

#endif
