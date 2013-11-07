//! @file rcc.c
//! @brief Gestion Reset et Clock
//! @author Atlantronic

#include "kernel/rcc.h"
#include "kernel/module.h"
#include "kernel/cpu/cpu.h"

// PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N 
#define PLL_M      8
#define PLL_N      336
// SYSCLK = PLL_VCO / PLL_P
#define PLL_P      2
// USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ
#define PLL_Q      7

static int rcc_module_init()
{
	// reset de la configuration pour éviter tout problème
	// HSION
	RCC->CR |= (uint32_t)0x00000001;

	// reset CFGR
	RCC->CFGR = 0x00;

	// reset HSEON, CSSON and PLLON bits
	RCC->CR &= (uint32_t)0xFEF6FFFF;

	// reset PLLCFGR
	RCC->PLLCFGR = 0x24003010;

	// reset HSEBYP bit
	RCC->CR &= (uint32_t)0xFFFBFFFF;

	// disable all interrupts
	RCC->CIR = 0x00;

	// configuration

	// Activation HSE et attente de HSE
	RCC->CR |= RCC_CR_HSEON;
	while((RCC->CR & RCC_CR_HSERDY) == 0)
	{

	}

	// Mode haute performances pour le passage a 168 MHz
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_PMODE;

	// HCLK = SYSCLK / 1
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

	// PCLK2 = HCLK / 2
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

	// PCLK1 = HCLK / 4
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;

	// PLL
	RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) | (RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);

	// activation PLL
	RCC->CR |= RCC_CR_PLLON;

	// attente PLL
	while((RCC->CR & RCC_CR_PLLRDY) == 0)
	{

	}

	// flash : utilisation du Prefetch
	FLASH->ACR = FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;

	// SYSCLK = PLL
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	// Attente de SYSCLK = PLLCLK
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
	{

	}

	// activation du FPU
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));

	return 0;
}

module_init(rcc_module_init, INIT_RCC);

// on doit la compiler en O2 pour avoir le bon nombre de cycles
__attribute__((optimize("-O2"))) void wait_active(uint32_t tick)
{
	tick >>= 2;
	for( ; tick-- ; )
	{
		nop();
	}
}

