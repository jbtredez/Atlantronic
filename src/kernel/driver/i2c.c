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

#if ! defined(__disco__)
#error unknown card
#endif

static xSemaphoreHandle i2c_mutex;
static xSemaphoreHandle i2c_sem;
static uint8_t* i2c_tx_buffer;
static int i2c_tx_buffer_size;
static int i2c_tx_buffer_count;
static uint8_t* i2c_rx_buffer;
static int i2c_rx_buffer_size;
static int i2c_rx_buffer_count;
static uint16_t i2c_addr;
static I2c_error i2c_error;

int i2c_module_init()
{
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

	NVIC_EnableIRQ(I2C3_EV_IRQn);
	NVIC_EnableIRQ(I2C3_ER_IRQn);
	NVIC_SetPriority(I2C3_EV_IRQn, PRIORITY_IRQ_I2C3_EV);
	NVIC_SetPriority(I2C3_ER_IRQn, PRIORITY_IRQ_I2C3_ER);

	i2c_mutex = xSemaphoreCreateMutex();
	vSemaphoreCreateBinary(i2c_sem);
	xSemaphoreTake(i2c_sem, 0);

	return 0;
}

module_init(i2c_module_init, INIT_I2C);

static void i2c_start_isr()
{
	if( i2c_tx_buffer_count < i2c_tx_buffer_size )
	{
		// start, on va ecrire l'adresse d'ecriture
		I2C3->DR = i2c_addr & ~I2C_OAR1_ADD0;
	}
	else
	{
		// start (ou restart), on va ecrire l'adresse de lecture
		I2C3->DR = i2c_addr | I2C_OAR1_ADD0;
	}
	I2C3->CR2 |= I2C_CR2_ITBUFEN;
}

static void i2c_addr_isr()
{
	if( i2c_tx_buffer_count == i2c_tx_buffer_size && i2c_rx_buffer_size == 1)
	{
		// desactivation ACK
		I2C3->CR1 &= ~I2C_CR1_ACK;

		// clear addr flag
		I2C3->SR1;
		I2C3->SR2;

		// stop
		I2C3->CR1 |= I2C_CR1_STOP;
	}
	else if( i2c_tx_buffer_count == i2c_tx_buffer_size && i2c_rx_buffer_size == 2 )
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
}

static long i2c_master_transmit_isr()
{
	long xHigherPriorityTaskWoken = 0;

	uint32_t btf = I2C3->SR1 & I2C_SR1_BTF;
	if( (I2C3->SR1 & I2C_SR1_TXE) && (I2C3->CR2 & I2C_CR2_ITBUFEN) && ! btf )
	{
		// TXE = 1 et BTF = 0
		if( i2c_tx_buffer_count < i2c_tx_buffer_size )
		{
			I2C3->DR = i2c_tx_buffer[i2c_tx_buffer_count];
			i2c_tx_buffer_count++;
		}
		else
		{
			I2C3->CR2 &= ~I2C_CR2_ITBUFEN;
		}
	}
	else if( btf && (I2C3->CR2 & I2C_CR2_ITEVTEN) )
	{
		// BTF = 1
		if( i2c_tx_buffer_count < i2c_tx_buffer_size )
		{
			// il reste des choses a envoyer
			I2C3->DR = i2c_tx_buffer[i2c_tx_buffer_count];
			i2c_tx_buffer_count++;
		}
		else if( i2c_rx_buffer_size )
		{
			// on a tout envoye et on va commencer a lire la reponse : restart
			I2C3->CR1 |= I2C_CR1_START;
		}
		else
		{
			// on a tout envoye et on n'attend rien en retour : stop
			I2C3->CR1 |= I2C_CR1_STOP;
			I2C3->CR2 &= ~(I2C_CR2_ITBUFEN | I2C_CR2_ITERREN | I2C_CR2_ITEVTEN);
			xSemaphoreGiveFromISR(i2c_sem, &xHigherPriorityTaskWoken);
		}
	}

	return xHigherPriorityTaskWoken;
}

static long i2c_master_receive_isr()
{
	long xHigherPriorityTaskWoken = 0;
	uint32_t btf = I2C3->SR1 & I2C_SR1_BTF;
	int rx_remain = i2c_rx_buffer_size - i2c_rx_buffer_count;

	if( (I2C3->SR1 & I2C_SR1_RXNE) && (I2C3->CR2 & I2C_CR2_ITBUFEN) && ! btf)
	{
		// RXNE = 1 et BTF = 0
		if( rx_remain > 3 )
		{
			i2c_rx_buffer[i2c_rx_buffer_count] = I2C3->DR;
			i2c_rx_buffer_count++;
		}
		else if( rx_remain == 2 || rx_remain == 3)
		{
			I2C3->CR2 &= ~I2C_CR2_ITBUFEN;
		}
		else
		{
			I2C3->CR2 &= ~(I2C_CR2_ITBUFEN | I2C_CR2_ITERREN | I2C_CR2_ITEVTEN);
			i2c_rx_buffer[i2c_rx_buffer_count] = I2C3->DR;
			i2c_rx_buffer_count++;
			xSemaphoreGiveFromISR(i2c_sem, &xHigherPriorityTaskWoken);
		}
	}
	else if( btf && (I2C3->CR2 & I2C_CR2_ITEVTEN) )
	{
		// BTF = 1
		if( rx_remain == 3 )
		{
			// desactivation ACK
			I2C3->CR1 &= ~I2C_CR1_ACK;
			i2c_rx_buffer[i2c_rx_buffer_count] = I2C3->DR;
			i2c_rx_buffer_count++;
		}
		else if( rx_remain == 2)
		{
			// stop
			I2C3->CR1 |= I2C_CR1_STOP;

			// lecture des 2 derniers octets dans le buffer
			i2c_rx_buffer[i2c_rx_buffer_count] = I2C3->DR;
			i2c_rx_buffer_count++;
			i2c_rx_buffer[i2c_rx_buffer_count] = I2C3->DR;
			i2c_rx_buffer_count++;
			I2C3->CR2 &= ~(I2C_CR2_ITBUFEN | I2C_CR2_ITERREN | I2C_CR2_ITEVTEN);
			xSemaphoreGiveFromISR(i2c_sem, &xHigherPriorityTaskWoken);
		}
		else
		{
			i2c_rx_buffer[i2c_rx_buffer_count] = I2C3->DR;
			i2c_rx_buffer_count++;
			if( rx_remain == 1 )
			{
				I2C3->CR2 &= ~(I2C_CR2_ITBUFEN | I2C_CR2_ITERREN | I2C_CR2_ITEVTEN);
				xSemaphoreGiveFromISR(i2c_sem, &xHigherPriorityTaskWoken);
			}
		}
	}
	return xHigherPriorityTaskWoken;
}

void isr_i2c3_ev(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;
	portSET_INTERRUPT_MASK_FROM_ISR();

	if( I2C3->SR2 & I2C_SR2_MSL )
	{
		if( I2C3->SR1 & I2C_SR1_SB )
		{
			i2c_start_isr();
		}
		else if( I2C3->SR1 & I2C_SR1_ADDR)
		{
			i2c_addr_isr();
		}
		else if( I2C3->SR2 & I2C_SR2_TRA)
		{
			xHigherPriorityTaskWoken = i2c_master_transmit_isr();
		}
		else
		{
			xHigherPriorityTaskWoken = i2c_master_receive_isr();
		}
	}
	else
	{
		// mode esclave, non implemente
	}

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

void isr_i2c3_er(void)
{
	uint32_t erren = I2C3->CR2 & I2C_CR2_ITERREN;
	if( (I2C3->SR1 & I2C_SR1_BERR) && erren )
	{
		// erreur BERR
		i2c_error = I2C_ERROR_BERR;
		I2C3->SR1 = ~I2C_SR1_BERR;
	}

	if( (I2C3->SR1 & I2C_SR1_ARLO) && erren )
	{
		// erreur ARLO (Arbitration lost)
		i2c_error = I2C_ERROR_ARLO;
		I2C3->SR1 = ~I2C_SR1_ARLO;
	}

	if( (I2C3->SR1 & I2C_SR1_AF) && erren )
	{
		if( ! (I2C3->SR2 & I2C_SR2_MSL) )
		{
			// cas slave non implemente
		}
		else
		{
			// erreur AF (acknowledge failure)
			i2c_error = I2C_ERROR_AF;
			I2C3->SR1 = ~I2C_SR1_AF;
		}
	}

	if( I2C3->SR1 & I2C_SR1_OVR && erren )
	{
		// erreur OVR
		i2c_error = I2C_ERROR_OVR;
		I2C3->SR1 = ~I2C_SR1_OVR;
	}
}

I2c_error i2c_transaction(uint16_t addr, void* tx_data, uint16_t tx_size, void* rx_data, uint16_t rx_size, uint32_t timeout)
{
	I2c_error res = I2C_ERROR_NONE;

	if( I2C3->SR2 & I2C_SR2_BUSY )
	{
		// busy
		res = I2C_ERROR_BUSY;
		goto done;
	}

	xSemaphoreTake(i2c_mutex, portMAX_DELAY);
	xSemaphoreTake(i2c_sem, 0);

	// activation des IT
	I2C3->CR1 &= ~I2C_CR1_POS;
	I2C3->CR2 |= I2C_CR2_ITBUFEN | I2C_CR2_ITERREN | I2C_CR2_ITEVTEN;

	i2c_tx_buffer = (uint8_t*)tx_data;
	i2c_tx_buffer_size = tx_size;
	i2c_rx_buffer = (uint8_t*) rx_data;
	i2c_rx_buffer_size = rx_size;
	i2c_tx_buffer_count = 0;
	i2c_rx_buffer_count = 0;
	i2c_error = I2C_ERROR_NONE;
	i2c_addr = addr;

	if( rx_size )
	{
		// enable ACK
		I2C3->CR1 |= I2C_CR1_ACK;
	}
	else
	{
		I2C3->CR1 &= ~I2C_CR1_ACK;
	}

	// start
	I2C3->CR1 |= I2C_CR1_START;
	if( xSemaphoreTake(i2c_sem, timeout) == pdFALSE )
	{
		log_format(LOG_ERROR, "i2c timeout tx %d / %d rx %d / %d", i2c_tx_buffer_count, i2c_tx_buffer_size, i2c_rx_buffer_count, i2c_rx_buffer_size);
		res = I2C_ERROR_TIMEOUT;
	}
	else
	{
		res = i2c_error;
	}

	xSemaphoreGive(i2c_mutex);

done:
	return res;
}
