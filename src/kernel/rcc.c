//! @file rcc.c
//! @brief Gestion Reset et Clock
//! @author Atlantronic

#include "kernel/rcc.h"
#include "kernel/module.h"
#include "kernel/cpu/cpu.h"

#ifdef STM32F4XX
// PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N 
#define PLL_M      8
#define PLL_N      336
// SYSCLK = PLL_VCO / PLL_P
#define PLL_P      2
// USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ
#define PLL_Q      7
#endif

static int rcc_module_init()
{
	// reset de la configuration pour éviter tout problème
	// HSION
	RCC->CR |= (uint32_t)0x00000001;

#ifdef STM32F10X_CL
	// reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits
	RCC->CFGR &= (uint32_t)0xF0FF0000;

	// reset HSEON, CSSON and PLLON bits
	RCC->CR &= (uint32_t)0xFEF6FFFF;

	// reset HSEBYP bit
	RCC->CR &= (uint32_t)0xFFFBFFFF;

	// reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits
	RCC->CFGR &= (uint32_t)0xFF80FFFF;

	// reset PLL2ON and PLL3ON bits
	RCC->CR &= (uint32_t)0xEBFFFFFF;

	// disable all interrupts and clear pending bits
	RCC->CIR = (uint32_t)0x00FF0000;

	// reset CFGR2 register
	RCC->CFGR2 = (uint32_t)0x00000000;
#endif

#ifdef STM32F4XX
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
#endif

	// configuration

	// Activation HSE et attente de HSE
	RCC->CR |= RCC_CR_HSEON;
	while((RCC->CR & RCC_CR_HSERDY) == 0)
	{

	}

#ifdef STM32F10X_CL
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
#endif

#ifdef STM32F4XX
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
#endif

	// Attente de SYSCLK = PLLCLK
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
	{

	}

#ifdef STM32F4XX
	// activation du FPU
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));
#endif

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

