#include "i2c.h"
#include "kernel/module.h"
#include "kernel/cpu/cpu.h"
#include "kernel/log.h"
#include "kernel/driver/usb.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "priority.h"
#include "gpio.h"
#include <math.h>

#define I2C_RISE_TIME(__FREQRANGE__, __SPEED__)    (((__SPEED__) <= 100000) ? ((__FREQRANGE__) + 1) : ((((__FREQRANGE__) * 300) / 1000) + 1))

static xSemaphoreHandle i2c_mutex;

int i2c_module_init()
{
#if defined(__disco__)
	// activation GPIOA, GPIOC et i2c3
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;

	gpio_pin_init(GPIOA, 8, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // SCL
	gpio_pin_init(GPIOC, 9, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // SDA

	gpio_af_config(GPIOA, 8, GPIO_AF_I2C3);
	gpio_af_config(GPIOC, 9, GPIO_AF_I2C3);

	// activation I2C3
	RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;

	// reset i2c3
	RCC->APB1RSTR |= RCC_APB1RSTR_I2C3RST;
	RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C3RST;

#if( RCC_PCLK1_MHZ != 48)
#error revoir i2c
#endif

	// desactivation i2c3
	I2C3->CR1 = 0;
	I2C3->CR2 = RCC_PCLK1_MHZ;
	// i2c_speed = PCLK1 / (25 * CCR) si fm et DUTY = 1
	// => pour 42MHz et 400k, CCR=4.2. On prend donc CCR=5 et du coup, i2c_speed = 384kHz
	I2C3->CCR = 5 | I2C_CCR_DUTY | I2C_CCR_FS;
	I2C3->TRISE = I2C_RISE_TIME(RCC_PCLK1_MHZ, 384000);
	// adresses 7 bits
	I2C3->OAR1 = 0x4000; // le bit 14 doit etre a 1
	I2C3->OAR2 = 0;

	I2C3->CR1 |= I2C_CR1_PE;

	// TODO passer le code en IT
	/*NVIC_EnableIRQ(I2C3_EV_IRQn);
	NVIC_EnableIRQ(I2C3_ER_IRQn);
	NVIC_SetPriority(I2C3_EV_IRQn, PRIORITY_IRQ_I2C3_EV);
	NVIC_SetPriority(I2C3_ER_IRQn, PRIORITY_IRQ_I2C3_ER);
*/
#else
#error unknown card
#endif

	i2c_mutex = xSemaphoreCreateMutex();

	return 0;
}

module_init(i2c_module_init, INIT_I2C);

I2c_error i2c_transaction(uint16_t i2c_addr, void* _tx_data, uint16_t tx_size, void* _rx_data, uint16_t rx_size, uint32_t timeout)
{
	I2c_error res = I2C_ERROR_NONE;
	uint8_t* tx_data = (uint8_t*)_tx_data;
	uint8_t* rx_data = (uint8_t*)_rx_data;

	if( I2C3->SR2 & I2C_SR2_BUSY )
	{
		// busy
		res = I2C_ERROR_BUSY;
		goto done;
	}

	xSemaphoreTake(i2c_mutex, portMAX_DELAY);

	if( rx_size )
	{
		// enable ACK
		I2C3->CR1 |= I2C_CR1_ACK;
	}

	// start
	I2C3->CR1 |= I2C_CR1_START;
	while( ! (I2C3->SR1 & I2C_SR1_SB) ) ; // TODO timeout

	// adresse en ecriture
	I2C3->DR = i2c_addr & ~I2C_OAR1_ADD0;
	while( !(I2C3->SR1 & I2C_SR1_ADDR) ) // TODO timeout
	{
		if( I2C3->SR1 & I2C_SR1_AF )
		{
			// Acknowledge Failure
			// => envoi stop
			I2C3->CR1 |= I2C_CR1_STOP;

			// clear AF
			I2C3->SR1 &= ~I2C_SR1_AF;
			res = I2C_ERROR_AF;
			goto unlock;
		}
	}

	// clear addr flag
	I2C3->SR1;
	I2C3->SR2;

	while( tx_size > 0 )
	{
		// attente TXE
		while( ! (I2C3->SR1 & I2C_SR1_TXE) ) ; // TODO timeout
		I2C3->DR = *tx_data;
		tx_data++;
		tx_size--;

		if( I2C3->SR1 & I2C_SR1_BTF && tx_size > 0)
		{
			I2C3->DR = *tx_data;
			tx_data++;
			tx_size--;
		}
	}

	// attente TXE
	while( ! (I2C3->SR1 & I2C_SR1_TXE) ) ; // TODO timeout

	if( rx_size == 0 )
	{
		// stop
		I2C3->CR1 |= I2C_CR1_STOP;

		goto wait_busy;
	}

	// restart
	I2C3->CR1 |= I2C_CR1_START;
	while( ! (I2C3->SR1 & I2C_SR1_SB) ) ; // TODO timeout

	// adresse en lecture
	I2C3->DR = i2c_addr | I2C_OAR1_ADD0;
	while( !(I2C3->SR1 & I2C_SR1_ADDR) ) // TODO timeout
	{
		if( I2C3->SR1 & I2C_SR1_AF )
		{
			// Acknowledge Failure
			// => envoi stop
			I2C3->CR1 |= I2C_CR1_STOP;

			// clear AF
			I2C3->SR1 &= ~I2C_SR1_AF;
			res = I2C_ERROR_AF;
			goto unlock;
		}
	}

	if( rx_size == 1 )
	{
		// desactivation ACK
		I2C3->CR1 &= ~I2C_CR1_ACK;

		// clear addr flag
		I2C3->SR1;
		I2C3->SR2;

		// stop
		I2C3->CR1 |= I2C_CR1_STOP;
	}
	else if( rx_size == 2 )
	{
		// desactivation ACK
		I2C3->CR1 &= ~I2C_CR1_ACK;

		// activation POS
		I2C3->CR1 |= I2C_CR1_POS;

		// clear addr flag
		I2C3->SR1;
		I2C3->SR2;
	}
	else
	{
		// clear addr flag
		I2C3->SR1;
		I2C3->SR2;
	}

	while( rx_size )
	{
		if( rx_size <= 3 )
		{
			if( rx_size == 1 )
			{
				// attente RXNE
				while( ! (I2C3->SR1 & I2C_SR1_RXNE) ) ; // TODO timeout
				*rx_data = I2C3->DR;
				rx_data++;
				rx_size--;

			}
			else if( rx_size == 2 )
			{
				// attente BTF
				while( ! (I2C3->SR1 & I2C_SR1_BTF) ) ; // TODO timeout

				// stop
				I2C3->CR1 |= I2C_CR1_STOP;

				*rx_data = I2C3->DR;
				rx_data++;
				rx_size--;

				*rx_data = I2C3->DR;
				rx_data++;
				rx_size--;
			}
			else
			{
				// attente BTF
				while( ! (I2C3->SR1 & I2C_SR1_BTF) ) ; // TODO timeout

				// desactivation ACK
				I2C3->CR1 &= ~I2C_CR1_ACK;

				*rx_data = I2C3->DR;
				rx_data++;
				rx_size--;

				// attente BTF
				while( ! (I2C3->SR1 & I2C_SR1_BTF) ) ; // TODO timeout

				// stop
				I2C3->CR1 |= I2C_CR1_STOP;

				*rx_data = I2C3->DR;
				rx_data++;
				rx_size--;

				*rx_data = I2C3->DR;
				rx_data++;
				rx_size--;
			}
		}
		else
		{
			// attente RXNE
			while( ! (I2C3->SR1 & I2C_SR1_RXNE) ) ; // TODO timeout

			*rx_data = I2C3->DR;
			rx_data++;
			rx_size--;

			if( I2C3->SR1 & I2C_SR1_BTF )
			{
				*rx_data = I2C3->DR;
				rx_data++;
				rx_size--;
			}
		}
	}

	// desactivation de POS
	I2C3->CR1 &= ~I2C_CR1_POS;


wait_busy:
	// attente si busy
	while( I2C3->SR2 & I2C_SR2_BUSY ) ; // TODO timeout

unlock:
	xSemaphoreGive(i2c_mutex);

done:
	return res;
}
