#ifndef RCC_H
#define RCC_H

//! @file rcc.h
//! @brief Gestion Reset et Clock
//! @author Atlantronic


// configuration une fois le module rcc initialisé (attention, il faut modifier le code de rcc.c si on veut changer les valeurs) :
#define RCC_HSI           8000000LL
#define RCC_HSE          25000000LL
#define RCC_PREDIV2CLK    5000000LL
#define RCC_PLL2CLK      40000000LL
#define RCC_PREDIV1CLK    8000000LL
#define RCC_PLLCLK       72000000LL
#define RCC_SYSCLK       72000000LL
#define RCC_HCLK         72000000LL
#define RCC_PCLK1        36000000LL
#define RCC_PCLK2        72000000LL

#define ms_to_tick(a)    (a*RCC_SYSCLK/1000)

#endif
