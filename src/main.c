//! @file main.c
//! @brief Programme principal
//! @author Jean-Baptiste Trédez

#ifdef __GCC_POSIX__
	#include <time.h>
	#include <signal.h>
	#include <unistd.h>
	#include <pthread.h>
#endif

#include "cpu/cpu.h"
#include "FreeRTOS.h"
#include "task.h"
#include "module.h"
#include "log.h"

#ifdef __GCC_POSIX__
static struct sigaction action;
static int n_sig_int;

void* end_sched(void *arg)
{
	(void) arg;
	vTaskEndScheduler();

	return NULL;
}

static void signal_handler(int sig)
{
	(void) sig;
	n_sig_int++;
	if(n_sig_int > 2)
	{
		kill(getpid(),SIGKILL);
	}
	else
	{
		pthread_t id;
		pthread_create(&id,NULL,end_sched,NULL);
	}
}
#endif

// TODO déménager dans un fichier rcc
void reset_rcc(void)
{
#ifndef __GCC_POSIX__
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
	RCC->CIR = 0x009F0000;
 #else
	// Reset PLL2ON and PLL3ON bits
	RCC->CR &= (uint32_t)0xEBFFFFFF;

	// Disable all interrupts and clear pending bits
	RCC->CIR = 0x00FF0000;

	// Reset CFGR2 register
	RCC->CFGR2 = 0x00000000;
 #endif
#endif
}

void init_rcc(void)
{
	#ifndef __GCC_POSIX__
	reset_rcc();

	// Enable HSE
	RCC->CR |= ((uint32_t)RCC_CR_HSEON);

	// Wait till HSE is ready
	while((RCC->CR & RCC_CR_HSERDY) == 0)
	{

	}

	// Enable Prefetch Buffer
	FLASH->ACR |= FLASH_ACR_PRFTBE;

	// Flash 2 wait state
	FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
	FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;    

	// HCLK = SYSCLK
	RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;

	// PCLK2 = HCLK
	RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;

	// PCLK1 = HCLK
	RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV1;

 #ifdef STM32F10X_CL
	// PLL configuration: PLLCLK = HSE * 9 = 72 MHz
	RCC->CFGR &= (uint32_t)~(RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
	RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLMULL9); 
 #else    
	//  PLL configuration: PLLCLK = HSE * 9 = 72 MHz
	RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE |
			RCC_CFGR_PLLMULL));
	RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL9);
 #endif

	// Enable PLL
	RCC->CR |= RCC_CR_PLLON;

	// Wait till PLL is ready
	while((RCC->CR & RCC_CR_PLLRDY) == 0)
	{

	}

	// Select PLL as system clock source
	RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
	RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;

	// Wait till PLL is used as system clock source
	while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08)
	{

	}
#endif
}

void init_panic(int init)
{
	(void) init;

	//! @todo allumer une led
	while(1)
	{

	}
}

int main()
{
	#ifdef __GCC_POSIX__
	n_sig_int = 0;
	action.sa_handler = signal_handler;
	sigemptyset(&(action.sa_mask));
	if(sigaction(SIGINT, &action, NULL))
	{
		logerror("sigaction");
	}
	#endif

	init_rcc();

	int init = initModules();

	if(init)
	{		
		init_panic(init);
	}

	vTaskStartScheduler();

	// on n'arrive jamais ici sur un pic
	#ifdef __GCC_POSIX__
	exitModules();
	#endif

	return 0;
}

void vApplicationIdleHook()
{
	// si on utilise les coroutines, on va ordonancer le tout dans la tache idle
	#if(configUSE_CO_ROUTINES == 1)
		vCoRoutineSchedule();
	#endif

	// en simulation, histoire de ne pas pomper 100% du temps processeur, on met un nanosleep dans la tache idle
	#ifdef __GCC_POSIX__
		struct timespec xTimeToSleep, xTimeSlept;
		xTimeToSleep.tv_sec = 0;
		xTimeToSleep.tv_nsec = 1000;
		nanosleep( &xTimeToSleep, &xTimeSlept );
	#endif
}

