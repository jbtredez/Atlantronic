//! @file rcc.c
//! @brief Gestion Reset et Clock
//! @author Jean-Baptiste Trédez

#include "io/rcc.h"
#include "module.h"
#include "cpu/cpu.h"

static int rcc_module_init()
{
	// reset de la configuration pour éviter tout problème
	// HSION
	RCC->CR |= (uint32_t)0x00000001;

	// Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits
#ifndef STM32F10X_CL
	RCC->CFGR &= (uint32_t)0xF8FF0000;
#else
	RCC->CFGR &= (uint32_t)0xF0FF0000;
#endif

	// Reset HSEON, CSSON and PLLON bits
	RCC->CR &= (uint32_t)0xFEF6FFFF;

	// Reset HSEBYP bit
	RCC->CR &= (uint32_t)0xFFFBFFFF;

	// Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits
	RCC->CFGR &= (uint32_t)0xFF80FFFF;

#ifndef STM32F10X_CL
	// Disable all interrupts and clear pending bits
	RCC->CIR = (uint32_t)0x009F0000;
#else
	// Reset PLL2ON and PLL3ON bits
	RCC->CR &= (uint32_t)0xEBFFFFFF;

	// Disable all interrupts and clear pending bits
	RCC->CIR = (uint32_t)0x00FF0000;

	// Reset CFGR2 register
	RCC->CFGR2 = (uint32_t)0x00000000;
#endif


	// configuration

	// Activation HSE et attente de HSE
	RCC->CR |= RCC_CR_HSEON;
	while((RCC->CR & RCC_CR_HSERDY) == 0)
	{

	}

	// Utilisation du Prefetch Buffer obligatoire à 72Hz
	FLASH->ACR |= FLASH_ACR_PRFTBE;

	// Flash : 2 états d'attente vu que l'on veut SYSCLK = 72 MHz
	// (pas d'attente si SYSCLK < 24 MHz, 1 état d'attente si SYSCLK <= 48 Mhz, 2 si <= 72 MHz)
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= FLASH_ACR_LATENCY_2;    

	// USBOTGCLK = 48 MHz = 2/3 * PLLCLK donc bit OTGFSPRE à 0
	RCC->CFGR2 &= ~RCC_CFGR_OTGFSPRE;

	// HCLK = SYSCLK = 72 MHz
	RCC->CFGR &= ~RCC_CFGR_HPRE;
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

	// PCLK2 = HCLK = 72 MHz
	RCC->CFGR &= ~ RCC_CFGR_PPRE2;
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

	// PCLK1 = HCLK / 2 = 36 MHz
	RCC->CFGR &= ~ RCC_CFGR_PPRE1;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

	// ADCCLK = PCLK2 / 6 = 12 MHz
	RCC->CFGR &= ~RCC_CFGR_ADCPRE;
	RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;

	// PREDIV2 = HSE / 5 = 5 MHz
	RCC->CFGR2 &= ~RCC_CFGR2_PREDIV2;
	RCC->CFGR2 |= RCC_CFGR2_PREDIV2_DIV5;

	// PLL2 = PREDIV2 * 8 = 40 MHz
	RCC->CFGR2 &= ~RCC_CFGR2_PLL2MUL;
	RCC->CFGR2 |= RCC_CFGR2_PLL2MUL8;

	// Activation PLL2
	RCC->CR |= RCC_CR_PLL2ON;

	// Attente PLL2
	while((RCC->CR & RCC_CR_PLL2RDY) == 0)
	{

	}

	// PREDIV1 = PLL2 / 5 = 8 MHz
	// PLL = PREDIV1 * 9 = 72 MHz
	RCC->CFGR &= ~(RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
	RCC->CFGR2 &= ~(RCC_CFGR2_PREDIV1SRC | RCC_CFGR2_PREDIV1);
	RCC->CFGR2 |= RCC_CFGR2_PREDIV1SRC_PLL2 | RCC_CFGR2_PREDIV1_DIV5;
	RCC->CFGR |= RCC_CFGR_PLLSRC_PREDIV1 | RCC_CFGR_PLLMULL9;

	// Activation PLL
	RCC->CR |= RCC_CR_PLLON;

	// Attente PLL
	while((RCC->CR & RCC_CR_PLLRDY) == 0)
	{

	}

	// SYSCLK = PLLCLK = 72 MHz
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	// Attente de SYSCLK = PLLCLK
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
	{

	}

	return 0;
}

module_init(rcc_module_init, INIT_RCC);

