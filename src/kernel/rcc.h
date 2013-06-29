#ifndef RCC_H
#define RCC_H

//! @file rcc.h
//! @brief Gestion Reset et Clock
//! @author Atlantronic

#include <stdint.h>

#ifdef STM32F10X_CL
// configuration une fois le module rcc initialisé (attention, il faut modifier le code de rcc.c si on veut changer les valeurs) :
#define RCC_HSI           8000000
#define RCC_HSE          25000000
#define RCC_PREDIV2CLK    5000000
#define RCC_PLL2CLK      40000000
#define RCC_PREDIV1CLK    8000000
#define RCC_PLLCLK       72000000
#define RCC_SYSCLK       72000000
#define RCC_SYSCLK_MHZ         72
#define RCC_HCLK         72000000
#define RCC_PCLK1        36000000
#define RCC_PCLK2        72000000

#define tick_to_us(a)    ((uint64_t)(a)/(RCC_SYSCLK/1000000))
#endif

#ifdef STM32F4XX
#define RCC_SYSCLK      168000000
#define RCC_SYSCLK_MHZ        168
#define RCC_HCLK_MHZ          168
#define RCC_PCLK1_MHZ          42
#define RCC_PCLK2_MHZ          84
#endif

#define ms_to_tick(a)    (a)
#define ms_to_systick(a)    ((uint64_t)(a)*RCC_SYSCLK/1000)
#define us_to_systick(a)    ((uint64_t)(a)*RCC_SYSCLK/1000000)

void wait_active(uint32_t tick);

#endif
