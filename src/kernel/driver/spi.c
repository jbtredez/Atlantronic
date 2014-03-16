#include "spi.h"
#include "kernel/module.h"
#include "kernel/cpu/cpu.h"
#include "kernel/log.h"
#include "kernel/driver/usb.h"
#include "gpio.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/semphr.h"
#include "kernel/fault.h"
#include <math.h>


#define SPI_STACK_SIZE             300

// interface accelero
#define LIS302DL_READ_CMD             0x80
#define LIS302DL_MULTIPLEBYTE_CMD     0x40
#define LIS302DL_CTRL_REG1_ADDR       0x20
#define LIS302DL_OUT_X_ADDR           0x29
#define LIS302DL_OUT_Y_ADDR           0x2B
#define LIS302DL_OUT_Z_ADDR           0x2D
#define LIS302DL_DATARATE_400         0x80
#define LIS302DL_LOWPOWERMODE_ACTIVE  0x40
#define LIS302DL_XYZ_ENABLE           0x07

// interface gyro
#define ADXRS453_READ          0x80000000
#define ADXRS453_WRITE         0x40000000
#define ADXRS453_SENSOR_DATA   0x20000000

#define ADXRS453_REG_RATE            0x00
#define ADXRS453_REG_TEM             0x02
#define ADXRS453_REG_LOCST           0x04
#define ADXRS453_REG_HICST           0x06
#define ADXRS453_REG_QUAD            0x08
#define ADXRS453_REG_FAULT           0x0A
#define ADXRS453_REG_PID             0x0C
#define ADXRS453_REG_SN_HIGH         0x0E
#define ADXRS453_REG_SN_LOW          0x10

struct spi_accel_dma_xyz
{
	unsigned char reserved0;
	signed char x;
	unsigned char reserved1;
	signed char y;
	unsigned char reserved2;
	signed char z;
} __attribute__((packed));


static unsigned char spi_tx_buffer[8];
static unsigned char spi_rx_buffer[8];
static struct spi_accel_dma_xyz spi_accel_dma_xyz;

static int16_t spi_gyro_v_lsb;        //!< vitesse brute du gyro (non corrigée)
static float spi_gyro_v;              //!< vitesse angulaire vue par le gyro
static float spi_gyro_v_nonoise;      //!< vitesse angulaire nettoyée du bruit
static float spi_gyro_theta_euler;    //!< angle intégré par un schéma Euler explicite
static float spi_gyro_theta_simpson;  //!< angle intégré par un schéma Simpson
static float spi_gyro_bias;           //!< correction deviation du gyro
static float spi_gyro_scale;          //!< gain du gyro
static float spi_gyro_dead_zone;      //!< permet de créer une zone morte sur le gyro pour avoir une vraie immobilité
static uint32_t spi_gyro_dev_count;   //!< nombre de donnees utilisées pour le calcul de la correction
static int spi_gyro_calib_mode;

#include "kernel/math/simpson_integrator.h"
static SimpsonState simpsonState;

static xSemaphoreHandle spi_sem;

static void spi_task(void* arg);

static void spi_gyro_calibration_cmd(void* arg);
static void spi_gyro_set_position_cmd(void* arg);
static void spi_gyro_set_calibration_values_cmd(void* arg);

int spi_module_init()
{
	// activation du SPI1
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	// activation dma2
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	// activation GPIOA (SCK, MOSI et MISO sur le port A)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	// activation GPIOE (CS de l'accelero)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

	gpio_pin_init(GPIOA, 5, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // SCK
	gpio_pin_init(GPIOA, 6, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // MISO
	gpio_pin_init(GPIOA, 7, GPIO_MODE_AF, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // MOSI
	gpio_pin_init(GPIOE, 3, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // CS0 accelero
	gpio_pin_init(GPIOA, 8, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // CS2 gyro
//	gpio_pin_init(GPIOA, 15, GPIO_MODE_OUT, GPIO_SPEED_50MHz, GPIO_OTYPE_PP, GPIO_PUPD_DOWN); // CS1

	// on ne selectionne rien
	gpio_set_pin(GPIOE, 3);
	gpio_set_pin(GPIOA, 8);

	gpio_af_config(GPIOA, 5, GPIO_AF_SPI1);
	gpio_af_config(GPIOA, 6, GPIO_AF_SPI1);
	gpio_af_config(GPIOA, 7, GPIO_AF_SPI1);


	// reset SPI1
	RCC->APB2RSTR |= RCC_APB2RSTR_SPI1;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1;

	// spi full duplex, lignes dediees a sens unique
	// datasize = 8
	// clock polarity : low
	// clock phase : edge
	// selection esclave (NSS) : soft (=> SSM = 1 et SSI = 1)
	// prescaler : 16 (SPI_CR1_BR_0 | SPI_CR1_BR_1) => PCLk2/16 = 5.25 MHz
	// format LSBFIRST
	// spi maitre
	SPI1->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | SPI_CR1_BR_0 | SPI_CR1_BR_1;

	// activation du mode SPI
	SPI1->I2SCFGR &= ~SPI_I2SCFGR_I2SMOD;

	// crc polynomial
	SPI1->CRCPR = 7;

	// enable SPI
	SPI1->CR1 |= SPI_CR1_SPE;

	// activation DMA sur SPI1 et de l'IT d'erreur
	SPI1->CR2 = SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN | SPI_CR2_ERRIE;

	// taille mémoire d'une donnée : 8 bits
	// incrément automatique mémoire : 1
	// taille mémoire périph d'une donnée : 8 bits
	// incrément automatique mémoire périph : 0
	// transfert (écriture) : mem => mem périph
	// transfert (lecture) : mem périph => mem
	// SPI1_RX : DMA2, stream0 sur chan3
	// SPI1_TX : DMA2, stream3 sur chan3
	DMA2_Stream0->CR = DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_1 | DMA_SxCR_MINC | DMA_SxCR_TCIE;
	DMA2_Stream0->PAR = (uint32_t) &SPI1->DR;
	DMA2_Stream3->CR = DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_1 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE;
	DMA2_Stream3->PAR = (uint32_t) &SPI1->DR;

	NVIC_EnableIRQ(SPI1_IRQn);
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	NVIC_EnableIRQ(DMA2_Stream3_IRQn);
	NVIC_SetPriority(SPI1_IRQn, PRIORITY_IRQ_SPI);
	NVIC_SetPriority(DMA2_Stream0_IRQn, PRIORITY_IRQ_DMA2_STREAM0);
	NVIC_SetPriority(DMA2_Stream3_IRQn, PRIORITY_IRQ_DMA2_STREAM3);

	vSemaphoreCreateBinary(spi_sem);
	xSemaphoreTake(spi_sem, 0);

	xTaskCreate(spi_task, "spi", SPI_STACK_SIZE, NULL, PRIORITY_TASK_SPI, NULL);
	usb_add_cmd(USB_CMD_GYRO_CALIB, &spi_gyro_calibration_cmd);
	usb_add_cmd(USB_CMD_GYRO_SET_POSITION, &spi_gyro_set_position_cmd);
	usb_add_cmd(USB_CMD_GYRO_SET_CALIBRATION_VALUES, &spi_gyro_set_calibration_values_cmd);

	return 0;
}

module_init(spi_module_init, INIT_SPI);

void isr_dma2_stream0()
{
	portBASE_TYPE xHigherPriorityTaskWoken = 0;
	portSET_INTERRUPT_MASK_FROM_ISR();

	if( DMA2->LISR | DMA_LISR_TCIF0)
	{
		DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
		DMA2_Stream0->CR &= ~DMA_SxCR_EN;
		xSemaphoreGiveFromISR(spi_sem, &xHigherPriorityTaskWoken);
	}

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
}

void isr_dma2_stream3()
{
	if( DMA2->LISR | DMA_LISR_TCIF3)
	{
		DMA2->LIFCR |= DMA_LIFCR_CTCIF3;
		DMA2_Stream3->CR &= ~DMA_SxCR_EN;
	}
}

void isr_spi1(void)
{
	int status = SPI1->SR;
	if( status & (SPI_SR_OVR | SPI_SR_MODF | SPI_SR_CRCERR) )
	{
		// TODO : remonter erreur + log erreur

		// erreur MODF qui ne doit pas arriver (NSS soft et SSI = 1)
		// erreur CRC qui ne doit pas arriver (pas active)

		// on desactive le DMA de reception
		DMA2_Stream0->CR &= ~DMA_SxCR_EN;

		// clear overrun : lecture DR puis lecture SR
		SPI1->DR;
		SPI1->SR;
	}
}

static void spi_set_read_dma_buffer(unsigned char* buf)
{
	DMA2_Stream0->M0AR = (uint32_t) buf;
}

static void spi_set_read_dma_size(uint16_t size)
{
	DMA2_Stream0->NDTR = size;
	DMA2_Stream0->CR |= DMA_SxCR_EN;
}

static void spi_set_write_dma_buffer(unsigned char* buf)
{
	DMA2_Stream3->M0AR = (uint32_t) buf;
}

static void spi_send_dma_buffer(uint16_t size)
{
	DMA2_Stream3->NDTR = size;
	DMA2_Stream3->CR |= DMA_SxCR_EN;
}

//! @return -1 si timeout
//! @return 0 sinon
static int spi_transaction(uint8_t* tx_buffer, uint8_t* rx_buffer, uint8_t size)
{
	xSemaphoreTake(spi_sem, 0);
	spi_set_read_dma_buffer(rx_buffer);
	spi_set_read_dma_size(size);
	spi_set_write_dma_buffer(tx_buffer);
	spi_send_dma_buffer(size);
	if( xSemaphoreTake(spi_sem, 1) == pdFALSE )
	{
		return -1;
	}

	return 0;
}

static void spi_init_accelero()
{
	spi_tx_buffer[0] = LIS302DL_CTRL_REG1_ADDR;
	spi_tx_buffer[1] = LIS302DL_DATARATE_400 | LIS302DL_LOWPOWERMODE_ACTIVE | LIS302DL_XYZ_ENABLE;

	gpio_reset_pin(GPIOE, 3);
	spi_transaction(spi_tx_buffer, spi_rx_buffer, 2);
	gpio_set_pin(GPIOE, 3);
}

static int spi_gyro_send_data(uint32_t data, uint32_t* rxdata)
{
	spi_tx_buffer[0] = (data >> 24) & 0xff;
	spi_tx_buffer[1] = (data >> 16) & 0xff;
	spi_tx_buffer[2] = (data >> 8) & 0xff;
	spi_tx_buffer[3] = data & 0xff;

	gpio_reset_pin(GPIOA, 8);
	int res = spi_transaction(spi_tx_buffer, spi_rx_buffer, 4);
	gpio_set_pin(GPIOA, 8);

	*rxdata = ((spi_rx_buffer[0] << 24) | (spi_rx_buffer[1] << 16) | (spi_rx_buffer[2] << 8) | (spi_rx_buffer[3] << 0));
	return res;
}

//! @return 0 si ok
//! @return -1 sinon
static int spi_init_gyro()
{
	int res = 0;
	uint32_t data_gyro;

	// 100 ms pour init du gyro apres mise sous tension
	vTaskDelay(100);
	res = spi_gyro_send_data(0x20000003, &data_gyro);
	if( res )
	{
		goto done;
	}

	vTaskDelay(50);
	res = spi_gyro_send_data(ADXRS453_SENSOR_DATA, &data_gyro);
	if( res )
	{
		goto done;
	}

	vTaskDelay(50);
	res = spi_gyro_send_data(ADXRS453_SENSOR_DATA, &data_gyro);
	if( res )
	{
		goto done;
	}
	res = spi_gyro_send_data(ADXRS453_SENSOR_DATA, &data_gyro);
	if( res )
	{
		goto done;
	}

	spi_gyro_calib_mode = 1;
	portENTER_CRITICAL();
	spi_gyro_scale = 0.000218166156f;  // from datasheet
	spi_gyro_dead_zone = 0.f;           // conservative value
	portEXIT_CRITICAL();

done:
	return res;
}

static int spi_gyro_update(float dt, int calibration)
{
	uint32_t data_gyro;
	int error = 0;
	int res = spi_gyro_send_data(ADXRS453_SENSOR_DATA, &data_gyro);
	if( res == 0 && (data_gyro & 0xe0000000) == 0)
	{
		fault(FAULT_GYRO_DISCONNECTED, FAULT_CLEAR);
		if( ((data_gyro & 0xC000000) == 0x4000000) && ((data_gyro & 0x04) != 0x04) )
		{
			portENTER_CRITICAL();
			spi_gyro_v_lsb = (int16_t)((data_gyro >> 10) & 0xffff);
			portEXIT_CRITICAL();
			fault(FAULT_GYRO_ERROR, FAULT_CLEAR);
		}
		else
		{
			fault(FAULT_GYRO_ERROR, FAULT_ACTIVE);
			error = 1;
		}
	}
	else
	{
		// erreur
		fault(FAULT_GYRO_DISCONNECTED, FAULT_ACTIVE);
		error = 1;
	}

	if( ! calibration )
	{
		// en cas d'erreur, on utilise l'ancienne valeur de vitesse
		portENTER_CRITICAL();
		spi_gyro_v = ((float)spi_gyro_v_lsb - spi_gyro_bias) * spi_gyro_scale;
		if(spi_gyro_v < spi_gyro_dead_zone && spi_gyro_v > -spi_gyro_dead_zone)
		{
			spi_gyro_v_nonoise = 0.f;
		}
		else
		{
			spi_gyro_v_nonoise = spi_gyro_v;
		}
		spi_gyro_theta_euler += spi_gyro_v_nonoise * dt;
		simpson_set_derivative(&simpsonState, dt, spi_gyro_v_nonoise);
		simpson_compute(&simpsonState);
		spi_gyro_theta_simpson = simpson_get(&simpsonState);
		portEXIT_CRITICAL();
	}
	else
	{
		// en cas d'erreur, on ne fait rien
		if( ! error )
		{
			portENTER_CRITICAL();
			spi_gyro_bias = (spi_gyro_dev_count * spi_gyro_bias + spi_gyro_v_lsb) / (spi_gyro_dev_count + 1);
			spi_gyro_v = ((float)spi_gyro_v_lsb - spi_gyro_bias) * spi_gyro_scale;
			spi_gyro_v_nonoise = spi_gyro_v;
			spi_gyro_theta_euler = 0.f;
			spi_gyro_theta_simpson = 0.f;
			portEXIT_CRITICAL();
			spi_gyro_dev_count++;
		}
	}

	return res;
}

int32_t spi_gyro_get_raw_data()
{
	float data;

	portENTER_CRITICAL();
	data = (int32_t) spi_gyro_v_lsb;
	portEXIT_CRITICAL();

	return data;
}

float spi_gyro_get_omega()
{
	float data;

	portENTER_CRITICAL();
	data = spi_gyro_v;
	portEXIT_CRITICAL();

	return data;
}

float spi_gyro_get_theta_euler()
{
	float data;

	portENTER_CRITICAL();
	data = spi_gyro_theta_euler;
	portEXIT_CRITICAL();

	return data;
}

float spi_gyro_get_theta_simpson()
{
	float data;

	portENTER_CRITICAL();
	data = simpson_get(&simpsonState);
	portEXIT_CRITICAL();

	return data;
}

void spi_gyro_set_theta(float theta)
{
	portENTER_CRITICAL();
	spi_gyro_theta_euler = theta;
	simpson_reset(&simpsonState, theta);
	spi_gyro_theta_simpson = simpson_get(&simpsonState);
	portEXIT_CRITICAL();
	log_format(LOG_INFO, "gyro set theta to : %d mrad", (int)(1000*theta));
}

static void spi_task(void* arg)
{
	(void) arg;

	spi_init_accelero();

	spi_init_gyro();

	while(1)
	{
		spi_tx_buffer[0] = LIS302DL_OUT_X_ADDR | LIS302DL_READ_CMD | LIS302DL_MULTIPLEBYTE_CMD;
		gpio_reset_pin(GPIOE, 3);
		spi_transaction(spi_tx_buffer, (void*)&spi_accel_dma_xyz, 6);
		gpio_set_pin(GPIOE, 3);

		spi_gyro_update(0.003f, spi_gyro_calib_mode);

		//log_format(LOG_INFO, "ACCEL %d %d %d GYRO %d", spi_accel_dma_xyz.x, spi_accel_dma_xyz.y, spi_accel_dma_xyz.z, (int)(spi_gyro_theta * 180 / M_PI));

		vTaskDelay(3);
	}
}

void spi_gyro_calib(int cmd)
{
	float bias = 0.f;
	switch(cmd)
	{
		case GYRO_CALIBRATION_START:
			spi_gyro_calib_mode = 1;
			spi_gyro_dev_count = 0;
			portENTER_CRITICAL();
			spi_gyro_bias = 0;
			portEXIT_CRITICAL();
			log(LOG_INFO, "gyro start calibration");
			break;
		case GYRO_CALIBRATION_STOP:
			spi_gyro_calib_mode = 0;
			portENTER_CRITICAL();
			bias = spi_gyro_bias;
			portEXIT_CRITICAL();
			log_format(LOG_INFO, "gyro stop calibration : %d mLSB/s", (int)(1000*bias));
			break;
		default:
			break;
	}
}

void spi_gyro_calibration_cmd(void* arg)
{
	int* cmd = (int*) arg;
	spi_gyro_calib(*cmd);
}

void spi_gyro_set_position_cmd(void* arg)
{
	float* theta;
	memcpy(&theta, arg, 4);
	spi_gyro_set_theta(*theta);
}

void spi_gyro_set_calibration_values_cmd(void* arg)
{
	struct spi_gyro_cmd_set_calibration_values_arg *param = (struct spi_gyro_cmd_set_calibration_values_arg*)arg;
	portENTER_CRITICAL();
	spi_gyro_scale = param->scale;
	spi_gyro_bias = param->bias;
	spi_gyro_dead_zone = param->dead_zone;
	portEXIT_CRITICAL();
}
