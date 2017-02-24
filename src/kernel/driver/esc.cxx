#include "esc.h"

#include "kernel/module.h"
#include "kernel/rcc.h"
#include "kernel/FreeRTOS.h"
#include "kernel/systick.h"
#include "kernel/log.h"
#include "io.h"

#define ESP_PERIOD           22500  // 22.5ms

static float esc_val = 0;
static int esc_remain_count = ESP_PERIOD;

int esc_module_init()
{
	//////////// TESTS gyro sur timer
	// activation timer 7
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
	TIM7->CR1 = 0x00;//TIM_CR1_ARPE;

	// TIM7_CLK = RCC_PCLK2_MHZ / (PSC + 1) = 1Mhz
	TIM7->PSC = RCC_PCLK2_MHZ-1;

	// IT dans esc_remain_count
	TIM7->ARR = esc_remain_count;
	TIM7->DIER = TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM7_IRQn);
	NVIC_SetPriority(TIM7_IRQn, PRIORITY_IRQ_SPI);

	// activation
	TIM7->CR1 |= TIM_CR1_CEN;

	return 0;
}

module_init(esc_module_init, INIT_GYRO);

static Systime esc_t1;
static Systime esc_t2;
static Systime esc_dt1;
static Systime esc_dt2;
static Systime esc_dt;

extern "C"
{
void isr_tim7(void)
{
	// generation d'un signal PPM
	if( TIM7->SR | TIM_SR_UIF )
 	{
		esc_val = gpio_get(GPIO_1);
		if( esc_val < 0)
		{
			esc_val = 0;
		}
		if( esc_val > 1 )
		{
			esc_val = 1;
		}
		// TODO limitation a 25% pour le moment
		if( esc_val > 0.25 )
		{
			esc_val = 0.25;
		}

		if( ! esc_remain_count )
		{
			int val = 700 + 1000 * esc_val; // entre 0.7 et 1.7 ms
			TIM7->ARR = val;
			esc_remain_count = ESP_PERIOD - val;
			gpio_set(GPIO_11);
			Systime t = systick_get_time_from_isr();
			esc_dt1 = t - esc_t1;
			esc_t1 = t;
		}
		else
		{
			TIM7->ARR = esc_remain_count;
			esc_remain_count = 0;
			gpio_reset(GPIO_11);
			Systime t = systick_get_time_from_isr();
			esc_dt2 = t - esc_t2;
			esc_t2 = t;
			esc_dt = esc_t2 - esc_t1;
		}
		TIM7->SR &= ~TIM_SR_UIF;
	}
}
}
