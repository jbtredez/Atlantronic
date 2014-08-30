#include "sdram.h"
#include "kernel/module.h"
#include "kernel/cpu/cpu.h"
#include "gpio.h"
#include "kernel/rcc.h"

#if ! defined( __disco__ )
#error sdram only for disco board
#endif

#define SDRAM_LoadToActiveDelay     2 // TMRD: 2 cycles => 1 cycle = 1/84MHz = 11.90ns
#define SDRAM_ExitSelfRefreshDelay  6 // TXSR: min=70ns (6x11.90ns)
#define SDRAM_SelfRefreshTime       4 // TRAS: min=42ns (4x11.90ns) max=120k (ns)
#define SDRAM_RowCycleDelay         6 // TRC:  min=70 (6x11.90ns)
#define SDRAM_WriteRecoveryTime     2 // TWR:  min=1+ 7ns (1+1x11.90ns)
#define SDRAM_RPDelay               2 // TRP:  20ns => 2x11.90ns
#define SDRAM_RCDDelay              2 // TRCD: 20ns => 2x11.90ns
#define SDRAM_REFRESH_COUNT      1292 // (15.62 us x Freq) - 20

#define FMC_SDRAM_CMD_NORMAL_MODE             ((uint32_t)0x00000000)
#define FMC_SDRAM_CMD_CLK_ENABLE              ((uint32_t)0x00000001)
#define FMC_SDRAM_CMD_PALL                    ((uint32_t)0x00000002)
#define FMC_SDRAM_CMD_AUTOREFRESH_MODE        ((uint32_t)0x00000003)
#define FMC_SDRAM_CMD_LOAD_MODE               ((uint32_t)0x00000004)
#define FMC_SDRAM_CMD_SELFREFRESH_MODE        ((uint32_t)0x00000005)
#define FMC_SDRAM_CMD_POWERDOWN_MODE          ((uint32_t)0x00000006)

#define FMC_SDRAM_CMD_TARGET_BANK2            FMC_SDCMR_CTB2
#define FMC_SDRAM_CMD_TARGET_BANK1            FMC_SDCMR_CTB1
#define FMC_SDRAM_CMD_TARGET_BANK1_2          FMC_SDCMR_CTB2 | FMC_SDCMR_CTB1

#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)

static void sdram_write_cmd(uint32_t cmdMode, uint32_t cmdTarget, uint32_t autoRefresh, uint32_t modeRegisterDefinition);

static int sdram_module_init()
{
	RCC->AHB3ENR |= RCC_AHB3ENR_FMCEN;

	// activation des gpio B,C,D,E,F et G
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN | RCC_AHB1ENR_GPIOGEN;
/*
	+-------------------+--------------------+--------------------+--------------------+
	+                       SDRAM pins assignment                                      +
	+-------------------+--------------------+--------------------+--------------------+
	| PD0  <-> FMC_D2   | PE0  <-> FMC_NBL0  | PF0  <-> FMC_A0    | PG0  <-> FMC_A10   |
	| PD1  <-> FMC_D3   | PE1  <-> FMC_NBL1  | PF1  <-> FMC_A1    | PG1  <-> FMC_A11   |
	| PD8  <-> FMC_D13  | PE7  <-> FMC_D4    | PF2  <-> FMC_A2    | PG4  <-> FMC_B0    |
	| PD9  <-> FMC_D14  | PE8  <-> FMC_D5    | PF3  <-> FMC_A3    | PG5  <-> FMC_B1    |
	| PD10 <-> FMC_D15  | PE9  <-> FMC_D6    | PF4  <-> FMC_A4    | PG8  <-> FMC_SDCLK |
	| PD14 <-> FMC_D0   | PE10 <-> FMC_D7    | PF5  <-> FMC_A5    | PG15 <-> FMC_NCAS  |
	| PD15 <-> FMC_D1   | PE11 <-> FMC_D8    | PF11 <-> FMC_NRAS  |--------------------+
	+-------------------| PE12 <-> FMC_D9    | PF12 <-> FMC_A6    |
	                    | PE13 <-> FMC_D10   | PF13 <-> FMC_A7    |
	                    | PE14 <-> FMC_D11   | PF14 <-> FMC_A8    |
	                    | PE15 <-> FMC_D12   | PF15 <-> FMC_A9    |
	+-------------------+--------------------+--------------------+
	| PB5 <-> FMC_SDCKE1|
	| PB6 <-> FMC_SDNE1 |
	| PC0 <-> FMC_SDNWE |
	+-------------------+
*/
	gpio_pin_init(GPIOB, 5, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOB, 6, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);

	gpio_pin_init(GPIOC, 0, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);

	gpio_pin_init(GPIOD, 0, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOD, 1, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOD, 8, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOD, 9, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOD, 10, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOD, 14, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOD, 15, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);

	gpio_pin_init(GPIOE, 0, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOE, 1, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOE, 7, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOE, 8, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOE, 9, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOE, 10, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOE, 11, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOE, 12, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOE, 13, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOE, 14, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOE, 15, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);

	gpio_pin_init(GPIOF, 0, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOF, 1, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOF, 2, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOF, 3, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOF, 4, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOF, 5, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOF, 11, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOF, 12, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOF, 13, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOF, 14, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOF, 15, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);

	gpio_pin_init(GPIOG, 0, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOG, 1, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOG, 4, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOG, 5, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOG, 8, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);
	gpio_pin_init(GPIOG, 15, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_NOPULL);

	gpio_af_config(GPIOB, 5, GPIO_AF_FSMC);
	gpio_af_config(GPIOB, 6, GPIO_AF_FSMC);

	gpio_af_config(GPIOC, 0, GPIO_AF_FSMC);

	gpio_af_config(GPIOD, 0, GPIO_AF_FSMC);
	gpio_af_config(GPIOD, 1, GPIO_AF_FSMC);
	gpio_af_config(GPIOD, 8, GPIO_AF_FSMC);
	gpio_af_config(GPIOD, 9, GPIO_AF_FSMC);
	gpio_af_config(GPIOD, 10, GPIO_AF_FSMC);
	gpio_af_config(GPIOD, 14, GPIO_AF_FSMC);
	gpio_af_config(GPIOD, 15, GPIO_AF_FSMC);

	gpio_af_config(GPIOE, 0, GPIO_AF_FSMC);
	gpio_af_config(GPIOE, 1, GPIO_AF_FSMC);
	gpio_af_config(GPIOE, 7, GPIO_AF_FSMC);
	gpio_af_config(GPIOE, 8, GPIO_AF_FSMC);
	gpio_af_config(GPIOE, 9, GPIO_AF_FSMC);
	gpio_af_config(GPIOE, 10, GPIO_AF_FSMC);
	gpio_af_config(GPIOE, 11, GPIO_AF_FSMC);
	gpio_af_config(GPIOE, 12, GPIO_AF_FSMC);
	gpio_af_config(GPIOE, 13, GPIO_AF_FSMC);
	gpio_af_config(GPIOE, 14, GPIO_AF_FSMC);
	gpio_af_config(GPIOE, 15, GPIO_AF_FSMC);

	gpio_af_config(GPIOF, 0, GPIO_AF_FSMC);
	gpio_af_config(GPIOF, 1, GPIO_AF_FSMC);
	gpio_af_config(GPIOF, 2, GPIO_AF_FSMC);
	gpio_af_config(GPIOF, 3, GPIO_AF_FSMC);
	gpio_af_config(GPIOF, 4, GPIO_AF_FSMC);
	gpio_af_config(GPIOF, 5, GPIO_AF_FSMC);
	gpio_af_config(GPIOF, 11, GPIO_AF_FSMC);
	gpio_af_config(GPIOF, 12, GPIO_AF_FSMC);
	gpio_af_config(GPIOF, 13, GPIO_AF_FSMC);
	gpio_af_config(GPIOF, 14, GPIO_AF_FSMC);
	gpio_af_config(GPIOF, 15, GPIO_AF_FSMC);

	gpio_af_config(GPIOG, 0, GPIO_AF_FSMC);
	gpio_af_config(GPIOG, 1, GPIO_AF_FSMC);
	gpio_af_config(GPIOG, 4, GPIO_AF_FSMC);
	gpio_af_config(GPIOG, 5, GPIO_AF_FSMC);
	gpio_af_config(GPIOG, 8, GPIO_AF_FSMC);
	gpio_af_config(GPIOG, 15, GPIO_AF_FSMC);

	// initialisation fmc
	// horloge : 2 : SDCLK = HCLK / 2
	// burst : non
	// rpipe : 1 cycle
	FMC_Bank5_6->SDCR[0] = FMC_SDCR1_SDCLK_1 | FMC_SDCR1_RPIPE_0;
	// colonnes : 8 bits
	// lignes : 12 bits
	// donnees : 16 bits
	// 4 internal bank
	// CAS latency : 3 cycles
	// pas de protection en ecriture
	FMC_Bank5_6->SDCR[1] = FMC_SDCR2_NR_0 | FMC_SDCR2_MWID_0 | FMC_SDCR2_NB | FMC_SDCR2_CAS_0 | FMC_SDCR2_CAS_1;

	// timings
	FMC_Bank5_6->SDTR[0] =(uint32_t)((((SDRAM_RowCycleDelay)-1) << 12) |\
            (((SDRAM_RPDelay)-1) << 20)
            );
	FMC_Bank5_6->SDTR[1] = (uint32_t)(((SDRAM_LoadToActiveDelay)-1)  |\
            (((SDRAM_ExitSelfRefreshDelay)-1) << 4) |\
            (((SDRAM_SelfRefreshTime)-1) << 8)      |\
            (((SDRAM_WriteRecoveryTime)-1) <<16)    |\
            (((SDRAM_RCDDelay)-1) << 24)
            );

	// sequence d'init de la sdram
	// configuration de l'horloge
	sdram_write_cmd(FMC_SDRAM_CMD_CLK_ENABLE, FMC_SDCMR_CTB2, 1, 0);
	wait_active(ms_to_tick(1));

	// configuration precharge all
	sdram_write_cmd(FMC_SDRAM_CMD_PALL, FMC_SDCMR_CTB2, 1, 0);

	// autorefresh
	sdram_write_cmd(FMC_SDRAM_CMD_AUTOREFRESH_MODE, FMC_SDCMR_CTB2, 4, 0);

	// Program the external memory mode register
	sdram_write_cmd(FMC_SDRAM_CMD_LOAD_MODE, FMC_SDCMR_CTB2, 1, SDRAM_MODEREG_BURST_LENGTH_2 | SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL | SDRAM_MODEREG_CAS_LATENCY_3 | SDRAM_MODEREG_OPERATING_MODE_STANDARD | SDRAM_MODEREG_WRITEBURST_MODE_SINGLE);

	FMC_Bank5_6->SDRTR |= SDRAM_REFRESH_COUNT << 1;

	return 0;
}

module_init(sdram_module_init, INIT_SDRAM);

static void sdram_write_cmd(uint32_t cmdMode, uint32_t cmdTarget, uint32_t autoRefresh, uint32_t modeRegisterDefinition)
{
	FMC_Bank5_6->SDCMR =cmdMode | cmdTarget | ((autoRefresh-1) << 5) | (modeRegisterDefinition << 9);
	while(FMC_Bank5_6->SDSR & FMC_SDSR_BUSY) ;
}
