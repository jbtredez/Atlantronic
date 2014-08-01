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
	RCC->CR |= RCC_CR_HSION | RCC_CR_HSITRIM_4;

	// reset CFGR
	RCC->CFGR = 0x00;

	// reset HSEON, CSSON, PLLON et PLLI2S
	RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON | RCC_CR_PLLI2SON);

	// reset PLLCFGR
	RCC->PLLCFGR = 0;
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_4 | RCC_PLLCFGR_PLLN_6 | RCC_PLLCFGR_PLLN_7 | RCC_PLLCFGR_PLLQ_2;

	// reset PLLI2SCFGR
	RCC->PLLI2SCFGR = 0;
	RCC->PLLI2SCFGR |= RCC_PLLI2SCFGR_PLLI2SN_6 | RCC_PLLI2SCFGR_PLLI2SN_7 | RCC_PLLI2SCFGR_PLLI2SR_1;

	// reset HSEBYP bit
	RCC->CR &= ~RCC_CR_HSEBYP;

	// disable all interrupts
	RCC->CIR = 0x00;

	// configuration
	// Mode haute performances pour le passage a 168 MHz ou 180 MHz
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_PMODE;

	// Activation HSE et attente de HSE
	RCC->CR |= RCC_CR_HSEON;
	while((RCC->CR & RCC_CR_HSERDY) == 0)
	{

	}

	// PLL
	RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) | (RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);

	// activation PLL
	RCC->CR |= RCC_CR_PLLON;

	// attente PLL
	while((RCC->CR & RCC_CR_PLLRDY) == 0)
	{

	}

#ifdef STM32F429xx
	// activation du "over drive" pour le passage a 180 Mhz
//	PWR->CR |= PWR_CR_ODEN;
#endif

	// flash : utilisation du Prefetch
	FLASH->ACR = FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;

	// HCLK = SYSCLK / 1
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

	// PCLK2 = HCLK / 2
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

	// PCLK1 = HCLK / 4
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;

	// SYSCLK = PLL
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	// Attente de SYSCLK = PLLCLK
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
	{

	}

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

void reboot()
{
	SCB->AIRCR = (uint32_t)0x5FA << SCB_AIRCR_VECTKEY_Pos | SCB_AIRCR_SYSRESETREQ_Msk;
	while(1) ;
}
