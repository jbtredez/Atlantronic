#ifndef RCC_H
#define RCC_H

//! @file rcc.h
//! @brief Gestion Reset et Clock
//! @author Jean-Baptiste Trédez


// configuration une fois le module rcc initialisé (attention, il faut modifier le code de rcc.c si on veut changer les valeurs) :
#define RCC_HSI           8000
#define RCC_HSE          24000
#define RCC_PREDIV2CLK    5000
#define RCC_PLL2CLK      40000
#define RCC_PREDIV1CLK    8000
#define RCC_PLLCLK       72000
#define RCC_SYSCLK       72000
#define RCC_HCLK         72000
#define RCC_PCLK1        36000
#define RCC_PCLK2        72000

#endif
