#include "bsp.h"
#include "kernel/module.h"
#include "kernel/rcc.h"
#include "kernel/FreeRTOS.h"
#include "kernel/queue.h"
#include "kernel/systick.h"
#include "kernel/driver/usb.h"
#include "kernel/driver/gpio.h"
#include "kernel/log.h"

volatile uint32_t color;
volatile uint8_t gpio_go;
volatile uint8_t gpio_color_change_enable;
volatile uint8_t gpio_enable_go = 0;
static volatile struct systime gpio_color_change_time;

static xQueueHandle gpio_queue_go;

static void gpio_cmd_go(void* arg);
static void gpio_cmd_color(void* arg);

typedef struct
{
	GPIO_TypeDef* gpio;
	uint32_t pin;
	enum gpio_pupd pupd;
}BspIo;


// io sur les connecteurs io generiques
static const BspIo bsp_io[] =
{
	{GPIOD, 11, GPIO_PUPD_UP}, // PD11 : IN_1
	{GPIOB, 13, GPIO_PUPD_UP}, // PB13 : IN_2
	{GPIOB, 12, GPIO_PUPD_UP}, // PB12 : IN_3
	{GPIOD, 10, GPIO_PUPD_UP}, // PD10 : IN_4
	{GPIOD,  7, GPIO_PUPD_UP}, // PD7  : IN_5
	{GPIOB, 11, GPIO_PUPD_UP}, // PB11 : IN_6
	{GPIOC, 11, GPIO_PUPD_UP}, // PC11 : IN_7
	{GPIOD,  6, GPIO_PUPD_UP}, // PD6  : IN_8
	{GPIOC,  9, GPIO_PUPD_UP}, // PC9  : IN_9
	{GPIOC,  8, GPIO_PUPD_UP}, // PC8  : IN_10
	{GPIOB, 14, GPIO_PUPD_UP}, // PB14 : IN_11
	{GPIOB, 15, GPIO_PUPD_UP}, // PB15 : IN_12
	{GPIOE,  6, GPIO_PUPD_UP}, // PE6  : IN_13
	{GPIOE,  5, GPIO_PUPD_UP}, // PE5  : IN_14
	{GPIOB,  7, GPIO_PUPD_DOWN}, // bouton USR2
	{GPIOC, 14, GPIO_PUPD_DOWN}, // bouton USR1
	{GPIOD,  3, GPIO_PUPD_DOWN}, // bouton go
};

static int gpio_module_init(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;

	// "io entrees"
	unsigned int i;
	for(i = 0; i < sizeof(bsp_io)/sizeof(bsp_io[0]); i++)
	{
		// pull up pour les omron
		gpio_pin_init(bsp_io[i].gpio, bsp_io[i].pin, GPIO_MODE_IN, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, bsp_io[i].pupd);
	}

	color = COLOR_UNKNOWN;
	gpio_go = 0;
	gpio_queue_go = xQueueCreate(1, 0);
	gpio_color_change_enable = 1;

	usb_add_cmd(USB_CMD_GO, &gpio_cmd_go);
	usb_add_cmd(USB_CMD_COLOR, &gpio_cmd_color);

	// boutons en IT sur front montant : USR1 et USR2
	// boutons en IT sur front descendant sur le GO
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PD;
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI7_PB;
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI14_PC;
	EXTI->IMR |= EXTI_IMR_MR3 | EXTI_IMR_MR7 | EXTI_IMR_MR14;
	EXTI->RTSR |= EXTI_RTSR_TR7 | EXTI_RTSR_TR14;
	EXTI->FTSR |= EXTI_RTSR_TR3;

	NVIC_SetPriority(EXTI3_IRQn, PRIORITY_IRQ_EXTI3);
	NVIC_SetPriority(EXTI9_5_IRQn, PRIORITY_IRQ_EXTI9_5);
	NVIC_SetPriority(EXTI15_10_IRQn, PRIORITY_IRQ_EXTI15_10);
	NVIC_EnableIRQ(EXTI3_IRQn);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	return 0;
}

module_init(gpio_module_init, INIT_GPIO);

uint32_t gpio_get_state()
{
	uint32_t res = 0;

	unsigned int i = 0;
	for(i = 0; i < sizeof(bsp_io)/sizeof(bsp_io[0]); i++)
	{
		res |= gpio_get_pin(bsp_io[i].gpio, bsp_io[i].pin) << i;
	}

	res |= (gpio_go?1:0) << i; // GO

	return res;
}

void gpio_wait_go()
{
	xQueuePeek(gpio_queue_go, NULL, portMAX_DELAY);
}

static void gpio_cmd_go(void * arg)
{
	struct gpio_cmd_go_arg* cmd_arg = (struct gpio_cmd_go_arg*) arg;

	switch(cmd_arg->cmd)
	{
		case GPIO_CMD_ENABLE_GO:
			if( gpio_enable_go != 1 )
			{
				gpio_enable_go = 1;
				log(LOG_INFO, "enable GO");
			}
			break;
		case GPIO_CMD_GO:
			if(gpio_enable_go)
			{
				log(LOG_INFO, "usb go");
				gpio_go = 1;
				//setLed(LED_CPU_RED | LED_CPU_BLUE);
				systick_start_match();
				xQueueSend(gpio_queue_go, NULL, 0);
			}
			break;
		default:
			log_format(LOG_ERROR, "unknown go cmd %d", cmd_arg->cmd);
			break;
	}
}

static void gpio_cmd_color(void* arg)
{
	uint8_t new_color = *((uint8_t*) arg);
	if(gpio_go == 0 && gpio_color_change_enable)
	{
		if(new_color == COLOR_RED)
		{
			color = COLOR_RED;
			log(LOG_INFO, "couleur => rouge");
		}
		else
		{
			color = COLOR_YELLOW;
			log(LOG_INFO, "couleur => jaune");
		}
	}
}

void isr_exti3(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;

	portSET_INTERRUPT_MASK_FROM_ISR();

	if( EXTI->PR & EXTI_PR_PR3)
	{
		EXTI->PR = EXTI_PR_PR3;
		if( gpio_enable_go )
		{
			gpio_go = 1;
			systick_start_match_from_isr();
			xQueueSendFromISR(gpio_queue_go, NULL, &xHigherPriorityTaskWoken);
		}
	}
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

void isr_exti9_5(void)
{
	portSET_INTERRUPT_MASK_FROM_ISR();

	if( EXTI->PR & EXTI_PR_PR7)
	{
		EXTI->PR = EXTI_PR_PR7;
		// TODO action bouton USR2
	}

	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

void isr_exti15_10(void)
{
	portSET_INTERRUPT_MASK_FROM_ISR();

	if( EXTI->PR & EXTI_PR_PR14)
	{
		EXTI->PR = EXTI_PR_PR14;
		if(gpio_go == 0 && gpio_color_change_enable)
		{
			struct systime t = systick_get_time_from_isr();
			struct systime dt = timediff(t, gpio_color_change_time);
			if( dt.ms > 300)
			{
				gpio_color_change_time = t;
				if(color == COLOR_YELLOW)
				{
					color = COLOR_RED;
				}
				else
				{
					color = COLOR_YELLOW;
				}
			}
		}
	}

	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}
