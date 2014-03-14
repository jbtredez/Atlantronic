#include "spi.h"
#include "accelero.h"
#include "kernel/module.h"
#include "kernel/cpu/cpu.h"
#include "kernel/log.h"
#include "kernel/driver/usb.h"
#include "gpio.h"
#include "kernel/fault.h"
#include <math.h>

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

struct spi_accel_dma_xyz
{
	unsigned char reserved0;
	signed char x;
	unsigned char reserved1;
	signed char y;
	unsigned char reserved2;
	signed char z;
} __attribute__((packed));

static AcceleroState accelero_state;
static unsigned char accelero_get_cmd[6];
static struct spi_accel_dma_xyz spi_accel_dma_xyz;

static void accelero_spi_callback();

int accelero_module_init()
{
	accelero_state = ACCELERO_STATE_DISCONNECTED;
	spi_register_callback(SPI_DEVICE_ACCELERO, accelero_spi_callback);

	accelero_get_cmd[0] = LIS302DL_OUT_X_ADDR | LIS302DL_READ_CMD | LIS302DL_MULTIPLEBYTE_CMD;
	accelero_get_cmd[1] = 0;
	accelero_get_cmd[2] = 0;
	accelero_get_cmd[3] = 0;
	accelero_get_cmd[4] = 0;
	accelero_get_cmd[5] = 0;

	return 0;
}

module_init(accelero_module_init, INIT_ACCELERO);

static void accelero_spi_callback()
{
	if( accelero_state == ACCELERO_STATE_RUNNING )
	{
		spi_transaction(SPI_DEVICE_ACCELERO, accelero_get_cmd, (uint8_t*)&spi_accel_dma_xyz, 6);
		// log_format(LOG_INFO, "ACCEL %d %d %d", spi_accel_dma_xyz.x, spi_accel_dma_xyz.y, spi_accel_dma_xyz.z);
	}
	else
	{
		unsigned char accelero_tx_buffer[2];
		unsigned char accelero_rx_buffer[2];

		accelero_tx_buffer[0] = LIS302DL_CTRL_REG1_ADDR;
		accelero_tx_buffer[1] = LIS302DL_DATARATE_400 | LIS302DL_LOWPOWERMODE_ACTIVE | LIS302DL_XYZ_ENABLE;

		spi_transaction(SPI_DEVICE_ACCELERO, accelero_tx_buffer, accelero_rx_buffer, 2);
		accelero_state = ACCELERO_STATE_RUNNING;
	}
}
