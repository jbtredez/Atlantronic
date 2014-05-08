#ifndef GPIO_H
#define GPIO_H

//! @file gpio.h
//! @brief Gpio mapping
//! @author Atlantronic

#include "kernel/cpu/cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LED_CPU_RED      0x00004000
#define LED_CPU_BLUE     0x00008000
#define LED_EXT_BLUE     0x80000000
#define LED_EXT_GREEN    0x20000000
#define LED_EXT_ORANGE1  0x00100000
#define LED_EXT_ORANGE2  0x00040000
#define LED_EXT_RED      0x01000000

#define COLOR_RED         0
#define COLOR_YELLOW      1

#define GPIO_IN_1          0x01  // IN_1
#define GPIO_IN_2          0x02  // IN_2
#define GPIO_IN_3          0x04  // IN_3
#define GPIO_IN_4          0x08  // IN_4
#define GPIO_IN_5          0x10  // IN_5
#define GPIO_IN_6          0x20  // IN_6
#define GPIO_IN_7          0x40  // IN_7
#define GPIO_IN_8          0x80  // IN_8
#define GPIO_IN_9         0x100  // IN_9
#define GPIO_IN_10        0x200  // IN_10
#define GPIO_IN_11        0x400  // IN_11
#define GPIO_IN_12        0x800  // IN_12
#define GPIO_IN_13       0x1000  // IN_13
#define GPIO_IN_14       0x2000  // IN_14
#define GPIO_IN_BTN1     0x4000  // IN_BTN1 (etat pin)
#define GPIO_IN_BTN2     0x8000  // IN_BTN2 (etat pin)
#define GPIO_IN_GO      0x10000  // IN_GO (etat pin)
#define GPIO_GO         0x20000  // GO : match lance
#define GPIO_COLOR      0x40000  // color

enum gpio_mode
{
	GPIO_MODE_IN   = 0x00,
	GPIO_MODE_OUT  = 0x01,
	GPIO_MODE_AF   = 0x02,
	GPIO_MODE_AN   = 0x03
};

enum gpio_speed
{
	GPIO_SPEED_2MHz   = 0x00,
	GPIO_SPEED_25MHz  = 0x01,
	GPIO_SPEED_50MHz  = 0x02,
	GPIO_SPEED_100MHz = 0x03
};

enum gpio_otype
{
	GPIO_OTYPE_PP = 0x00, // push-pull
	GPIO_OTYPE_OD = 0x01  // open drain
};

enum gpio_pupd
{
	GPIO_PUPD_NOPULL = 0x00,
	GPIO_PUPD_UP     = 0x01,
	GPIO_PUPD_DOWN   = 0x02
};

#define GPIO_AF_RTC_50Hz      ((uint8_t)0x00)  /* RTC_50Hz Alternate Function mapping */
#define GPIO_AF_MCO           ((uint8_t)0x00)  /* MCO (MCO1 and MCO2) Alternate Function mapping */
#define GPIO_AF_TAMPER        ((uint8_t)0x00)  /* TAMPER (TAMPER_1 and TAMPER_2) Alternate Function mapping */
#define GPIO_AF_SWJ           ((uint8_t)0x00)  /* SWJ (SWD and JTAG) Alternate Function mapping */
#define GPIO_AF_TRACE         ((uint8_t)0x00)  /* TRACE Alternate Function mapping */

//!<AF 1 selection
#define GPIO_AF_TIM1          ((uint8_t)0x01)  /* TIM1 Alternate Function mapping */
#define GPIO_AF_TIM2          ((uint8_t)0x01)  /* TIM2 Alternate Function mapping */

//!<AF 2 selection
#define GPIO_AF_TIM3          ((uint8_t)0x02)  /* TIM3 Alternate Function mapping */
#define GPIO_AF_TIM4          ((uint8_t)0x02)  /* TIM4 Alternate Function mapping */
#define GPIO_AF_TIM5          ((uint8_t)0x02)  /* TIM5 Alternate Function mapping */

//!<AF 3 selection
#define GPIO_AF_TIM8          ((uint8_t)0x03)  /* TIM8 Alternate Function mapping */
#define GPIO_AF_TIM9          ((uint8_t)0x03)  /* TIM9 Alternate Function mapping */
#define GPIO_AF_TIM10         ((uint8_t)0x03)  /* TIM10 Alternate Function mapping */
#define GPIO_AF_TIM11         ((uint8_t)0x03)  /* TIM11 Alternate Function mapping */

//!<AF 4 selection
#define GPIO_AF_I2C1          ((uint8_t)0x04)  /* I2C1 Alternate Function mapping */
#define GPIO_AF_I2C2          ((uint8_t)0x04)  /* I2C2 Alternate Function mapping */
#define GPIO_AF_I2C3          ((uint8_t)0x04)  /* I2C3 Alternate Function mapping */

//!<AF 5 selection
#define GPIO_AF_SPI1          ((uint8_t)0x05)  /* SPI1 Alternate Function mapping */
#define GPIO_AF_SPI2          ((uint8_t)0x05)  /* SPI2/I2S2 Alternate Function mapping */

//!<AF 6 selection
#define GPIO_AF_SPI3          ((uint8_t)0x06)  /* SPI3/I2S3 Alternate Function mapping */

//!<AF 7 selection
#define GPIO_AF_USART1        ((uint8_t)0x07)  /* USART1 Alternate Function mapping */
#define GPIO_AF_USART2        ((uint8_t)0x07)  /* USART2 Alternate Function mapping */
#define GPIO_AF_USART3        ((uint8_t)0x07)  /* USART3 Alternate Function mapping */
#define GPIO_AF_I2S3ext       ((uint8_t)0x07)  /* I2S3ext Alternate Function mapping */

//!<AF 8 selection
#define GPIO_AF_UART4         ((uint8_t)0x08)  /* UART4 Alternate Function mapping */
#define GPIO_AF_UART5         ((uint8_t)0x08)  /* UART5 Alternate Function mapping */
#define GPIO_AF_USART6        ((uint8_t)0x08)  /* USART6 Alternate Function mapping */

//!<AF 9 selection
#define GPIO_AF_CAN1          ((uint8_t)0x09)  /* CAN1 Alternate Function mapping */
#define GPIO_AF_CAN2          ((uint8_t)0x09)  /* CAN2 Alternate Function mapping */
#define GPIO_AF_TIM12         ((uint8_t)0x09)  /* TIM12 Alternate Function mapping */
#define GPIO_AF_TIM13         ((uint8_t)0x09)  /* TIM13 Alternate Function mapping */
#define GPIO_AF_TIM14         ((uint8_t)0x09)  /* TIM14 Alternate Function mapping */

//!<AF 10 selection
#define GPIO_AF_OTG_FS         ((uint8_t)0xA)  /* OTG_FS Alternate Function mapping */
#define GPIO_AF_OTG_HS         ((uint8_t)0xA)  /* OTG_HS Alternate Function mapping */

//!<AF 11 selection
#define GPIO_AF_ETH             ((uint8_t)0x0B)  /* ETHERNET Alternate Function mapping */

//!<AF 12 selection
#define GPIO_AF_FSMC            ((uint8_t)0xC)  /* FSMC Alternate Function mapping */
#define GPIO_AF_OTG_HS_FS       ((uint8_t)0xC)  /* OTG HS configured in FS, Alternate Function mapping */
#define GPIO_AF_SDIO            ((uint8_t)0xC)  /* SDIO Alternate Function mapping */

//!<AF 13 selection
#define GPIO_AF_DCMI          ((uint8_t)0x0D)  /* DCMI Alternate Function mapping */

//!<AF 15 selection
#define GPIO_AF_EVENTOUT      ((uint8_t)0x0F)  /* EVENTOUT Alternate Function mapping */

#ifndef LINUX
void gpio_pin_init(GPIO_TypeDef* GPIOx, uint32_t pin, enum gpio_mode mode, enum gpio_speed speed, enum gpio_otype otype, enum gpio_pupd pupd);

void gpio_af_config(GPIO_TypeDef* GPIOx, uint32_t pin, uint32_t gpio_af);

void setLed(uint32_t mask);

static inline void gpio_set_pin(GPIO_TypeDef* GPIOx, uint32_t pin)
{
	GPIOx->BSRRL = 1 << pin;
};

static inline void gpio_reset_pin(GPIO_TypeDef* GPIOx, uint32_t pin)
{
	GPIOx->BSRRH = 1 << pin;
};

static inline uint32_t gpio_get_pin(GPIO_TypeDef* GPIOx, uint32_t pin)
{
	return (GPIOx->IDR >> pin) & 0x01;
};

static inline void gpio_power_on()
{
	gpio_reset_pin(GPIOB, 2);
}

static inline void gpio_power_off()
{
	gpio_set_pin(GPIOB, 2);
}

static inline uint32_t getcolor()
{
	extern volatile uint32_t color;
	return color;
}

static inline uint8_t getGo()
{
	extern volatile uint8_t gpio_go;
	return gpio_go;
}

uint32_t gpio_get_state();

void gpio_wait_go();
#endif
// ---------------- interface usb ------------
enum
{
	GPIO_CMD_ENABLE_GO,
	GPIO_CMD_GO,
};

struct gpio_cmd_go_arg
{
	uint8_t cmd;
};

#ifdef __cplusplus
}
#endif

#endif
