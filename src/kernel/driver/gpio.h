#ifndef GPIO_H
#define GPIO_H

//! @file gpio.h
//! @brief Gpio mapping
//! @author Atlantronic

#include "kernel/cpu/cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

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
#ifdef STM32F429xx
#define GPIO_AF_SPI4          ((uint8_t)0x05)  /* SPI4 Alternate Function mapping        */
#define GPIO_AF_SPI5          ((uint8_t)0x05)  /* SPI5 Alternate Function mapping        */
#define GPIO_AF_SPI6          ((uint8_t)0x05)  /* SPI6 Alternate Function mapping        */
#define GPIO_AF5_I2S3ext       ((uint8_t)0x05)  /* I2S3ext_SD Alternate Function mapping  */
#endif

//!<AF 6 selection
#define GPIO_AF_SPI3          ((uint8_t)0x06)  /* SPI3/I2S3 Alternate Function mapping */
#ifdef STM32F429xx
#define GPIO_AF_I2S2ext       ((uint8_t)0x06)  /* I2S2ext_SD Alternate Function mapping */
#define GPIO_AF_SAI1          ((uint8_t)0x06)  /* SAI1 Alternate Function mapping       */
#endif

//!<AF 7 selection
#define GPIO_AF_USART1        ((uint8_t)0x07)  /* USART1 Alternate Function mapping */
#define GPIO_AF_USART2        ((uint8_t)0x07)  /* USART2 Alternate Function mapping */
#define GPIO_AF_USART3        ((uint8_t)0x07)  /* USART3 Alternate Function mapping */
#define GPIO_AF7_I2S3ext       ((uint8_t)0x07)  /* I2S3ext Alternate Function mapping */

//!<AF 8 selection
#define GPIO_AF_UART4         ((uint8_t)0x08)  /* UART4 Alternate Function mapping */
#define GPIO_AF_UART5         ((uint8_t)0x08)  /* UART5 Alternate Function mapping */
#define GPIO_AF_USART6        ((uint8_t)0x08)  /* USART6 Alternate Function mapping */
#ifdef STM32F429xx
#define GPIO_AF_UART7         ((uint8_t)0x08)  /* UART7 Alternate Function mapping  */
#define GPIO_AF_UART8         ((uint8_t)0x08)  /* UART8 Alternate Function mapping  */
#endif

//!<AF 9 selection
#define GPIO_AF_CAN1          ((uint8_t)0x09)  /* CAN1 Alternate Function mapping */
#define GPIO_AF_CAN2          ((uint8_t)0x09)  /* CAN2 Alternate Function mapping */
#define GPIO_AF_TIM12         ((uint8_t)0x09)  /* TIM12 Alternate Function mapping */
#define GPIO_AF_TIM13         ((uint8_t)0x09)  /* TIM13 Alternate Function mapping */
#define GPIO_AF_TIM14         ((uint8_t)0x09)  /* TIM14 Alternate Function mapping */
#ifdef STM32F429xx
#define GPIO_AF9_LTDC          ((uint8_t)0x09)  /* LCD-TFT Alternate Function mapping */
#endif

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

//!<AF 14 selection
#ifdef STM32F429xx
#define GPIO_AF14_LTDC          ((uint8_t)0x0E)  /* LCD-TFT Alternate Function mapping */
#endif

//!<AF 15 selection
#define GPIO_AF_EVENTOUT      ((uint8_t)0x0F)  /* EVENTOUT Alternate Function mapping */

#ifndef LINUX
void gpio_pin_init(GPIO_TypeDef* GPIOx, uint32_t pin, enum gpio_mode mode, enum gpio_speed speed, enum gpio_otype otype, enum gpio_pupd pupd);

void gpio_af_config(GPIO_TypeDef* GPIOx, uint32_t pin, uint32_t gpio_af);

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

#endif

#ifdef __cplusplus
}
#endif

#endif
