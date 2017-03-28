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
	//////////// TESTS esc sur timer
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

extern "C"
{
void isr_tim7(void)
{
	// generation d'un signal PPM
	// signal periodique de periode ESP_PERIOD (22.5ms)
	// 1er esc signal haut pendant entre 0.7ms et 1.7ms (0.7ms = 0% et 1.7ms = 100%)
	// 2eme a 8eme esc : pas supporte (on laisse le signal bas)
	// signal bas sur le reste de la periode
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
		// TODO limitation a 45% pour le moment
		if( esc_val > 0.45 )
		{
			esc_val = 0.45;
		}

		if( ! esc_remain_count )
		{
			gpio_set(GPIO_11);

			int val = 700 + 1000 * esc_val; // entre 0.7 et 1.7 ms
			TIM7->ARR = val;
			esc_remain_count = ESP_PERIOD - val;
		}
		else
		{
			gpio_reset(GPIO_11);
			TIM7->ARR = esc_remain_count;
			esc_remain_count = 0;
		}
		TIM7->SR &= ~TIM_SR_UIF;
	}
}
}
