#ifndef RCC_H
#define RCC_H

//! @file rcc.h
//! @brief Gestion Reset et Clock
//! @author Atlantronic

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RCC_SYSCLK      168000000
#define RCC_SYSCLK_MHZ        168
#define RCC_HCLK_MHZ          168
#define RCC_PCLK1_MHZ          42
#define RCC_PCLK2_MHZ          84

#define ms_to_tick(a)    (a)
#define ms_to_systick(a)    ((uint64_t)(a)*RCC_SYSCLK/1000)
#define us_to_systick(a)    ((uint64_t)(a)*RCC_SYSCLK/1000000)

void wait_active(uint32_t tick);

void reboot(void);

#ifdef __cplusplus
}
#endif

#endif
