//! @file pwm.c
//! @brief PWM
//! @author Atlantronic

#define WEAK_PWM
#include "pwm.h"
#include "gpio.h"
#include "kernel/module.h"
#include "kernel/cpu/cpu.h"

void isr_pwm_reset(void);
static int pwm_on;

#define PWM_CH1         0x01
#define PWM_CH2         0x02
#define PWM_CH3         0x04
#define PWM_CH4         0x08

typedef struct
{
	volatile uint32_t* ccrx;
	GPIO_TypeDef* gpio_dir;
	uint32_t pin_dir;
	uint32_t arr;
} PwmMapping;

PwmMapping pwm_map[PWM_MAX];

static void pwm_timer_init(TIM_TypeDef* tim, uint32_t arr, uint32_t pwm_ch_mask);
static void pwn_pin_init(const unsigned int id, GPIO_TypeDef* GPIOx_ch, uint32_t pin_ch, volatile uint32_t* ccrx, uint32_t arr, uint32_t gpio_af, GPIO_TypeDef* gpio_dir, uint32_t pin_dir);
static void pwm_cmd(void* arg);

static int pwm_module_init()
{
	// activation GPIOA, GPIOC, GPIOE, GPIOF, GPIOG
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN | RCC_AHB1ENR_GPIOGEN;

	pwn_pin_init(PWM_1, GPIOA, 2, &TIM5->CCR3, PWM_ARR1, GPIO_AF_TIM5, GPIOF, 6);
	pwn_pin_init(PWM_2, GPIOA, 3, &TIM5->CCR4, PWM_ARR1, GPIO_AF_TIM5, GPIOC, 8);
	pwn_pin_init(PWM_3, GPIOE, 5, &TIM9->CCR1, PWM_ARR2, GPIO_AF_TIM9, GPIOG, 3);
	pwn_pin_init(PWM_4, GPIOE, 6, &TIM9->CCR2, PWM_ARR2, GPIO_AF_TIM9, GPIOG, 2);

	// activation clock sur le timer 5
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	pwm_timer_init(TIM5, PWM_ARR1, PWM_CH3 | PWM_CH4);

	// activation clock sur le timer 9
	RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
	pwm_timer_init(TIM9, PWM_ARR2, PWM_CH1 | PWM_CH2);

	usb_add_cmd(USB_CMD_PWM, pwm_cmd);

	pwm_on = 1;

	return 0;
}

module_init(pwm_module_init, INIT_PWM);

module_exit(isr_pwm_reset, EXIT_PWM);

static void pwn_pin_init(const unsigned int id, GPIO_TypeDef* GPIOx_ch, uint32_t pin_ch, volatile uint32_t* ccrx, uint32_t arr, uint32_t gpio_af, GPIO_TypeDef* gpio_dir, uint32_t pin_dir)
{
	if( id >= PWM_MAX )
	{
		return;
	}

	// pin pwm
	gpio_pin_init(GPIOx_ch, pin_ch, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);
	gpio_af_config(GPIOx_ch, pin_ch, gpio_af);

	// pin direction
	gpio_pin_init(gpio_dir, pin_dir, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_UP);

	pwm_map[id].ccrx = ccrx;
	pwm_map[id].gpio_dir = gpio_dir;
	pwm_map[id].pin_dir = pin_dir;
	pwm_map[id].arr = arr;
}

static void pwm_timer_init(TIM_TypeDef* tim, uint32_t arr, uint32_t pwm_ch_mask)
{
	tim->PSC = PWM_PSC;
	tim->ARR = arr;
	tim->RCR =0x00;
	tim->CR1 = 0x00;
	tim->CR2 = 0x00;
	tim->CR1 |= /*TIM_CR1_ARPE |*/ TIM_CR1_URS;
	tim->SMCR = 0x00;

	// mise à jour "update generation"
	tim->EGR |= TIM_EGR_UG;

	tim->CCER = 0x00; // permet de programmer CCMR1 et CCMR2
	if( tim == TIM1 || tim == TIM8 )
	{
		tim->BDTR = 0x00; // permet de programmer CCMR1 et CCMR2
	}

	uint32_t ccmr1 = 0;
	uint32_t ccmr2 = 0;
	uint32_t ccer = 0;

	if( pwm_ch_mask & PWM_CH1 )
	{
		ccmr1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE; // mode PWM 1 sur le canal 1 avec preload
		ccer |= TIM_CCER_CC1E;
		tim->CCR1 = 0x00; // pwm initiale a 0 sur le canal 1
	}

	if( pwm_ch_mask & PWM_CH2 )
	{
		ccmr1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;  // mode PWM 1 sur le canal 2 avec preload
		ccer |= TIM_CCER_CC2E;
		tim->CCR2 = 0x00; // pwm initiale a 0 sur le canal 2
	}

	if( pwm_ch_mask & PWM_CH3 )
	{
		ccmr2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE; // mode PWM 1 sur le canal 3 avec preload
		ccer |= TIM_CCER_CC3E;
		tim->CCR3 = 0x00; // pwm initiale a 0 sur le canal 3
	}

	if( pwm_ch_mask & PWM_CH4 )
	{
		ccmr2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;  // mode PWM 1 sur le canal 4 avec preload
		ccer |= TIM_CCER_CC4E;
		tim->CCR4 = 0x00; // pwm initiale a 0 sur le canal 4
	}

	tim->CCMR1 = ccmr1;
	tim->CCMR2 = ccmr2;
	tim->CCER = ccer;

	// on active le tout
	tim->CR1 |= TIM_CR1_CEN;
	if( tim == TIM1 || tim == TIM8 )
	{
		tim->BDTR |= TIM_BDTR_MOE;
	}
}

void pwm_set(const unsigned int id, float val)
{
	int dir = 1;

	if( id >= PWM_MAX )
	{
		log_format(LOG_ERROR, "unknown pwm id %d", id);
		return;
	}

	if( ! pwm_on )
	{
		val = 0;
	}

	if(val < 0)
	{
		val = -val;
		dir = -1;
	}

	int arr = pwm_map[id].arr;
	uint16_t val16 = val * arr;
	if( val16 > arr)
	{
		val16 = arr;
	}

	if(dir > 0)
	{
		gpio_set_pin(pwm_map[id].gpio_dir, pwm_map[id].pin_dir);
	}
	else
	{
		gpio_reset_pin(pwm_map[id].gpio_dir, pwm_map[id].pin_dir);
	}

	*pwm_map[id].ccrx = val16;
}

void isr_pwm_reset(void)
{
	// on est dans une IT d'erreur ou fin du match => arrêt des moteurs
	pwm_disable();
}

void pwm_enable()
{
	pwm_on = 1;
}

void pwm_disable()
{
	pwm_on = 0;
	int i = 0;
	for(i = 0; i < PWM_MAX; i++)
	{
		*pwm_map[i].ccrx = 0;
	}
}

static void pwm_cmd(void* arg)
{
	struct pwm_usb_cmd* cmd = (struct pwm_usb_cmd*) arg;

	pwm_set(cmd->id, cmd->val);
}
