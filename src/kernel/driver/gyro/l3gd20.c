#include "kernel/driver/spi.h"
#include "kernel/module.h"
#include "kernel/cpu/cpu.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/log.h"
#include "priority.h"

#define L3GD20_STACK_SIZE         300

#define L3GD20_READCMD                0x80
#define L3GD20_MULTIBYTECMD           0x40
#define L3GD20_ID_ADDR                0x0f
#define L3GD20_CTRL_REG1_ADDR         0x20
#define L3GD20_CTRL_REG2_ADDR         0x21
#define L3GD20_CTRL_REG3_ADDR         0x22
#define L3GD20_CTRL_REG4_ADDR         0x23
#define L3GD20_CTRL_REG5_ADDR         0x24
#define L3GD20_REFERENCE_REG_ADDR     0x25
#define L3GD20_OUT_TEMP_ADDR          0x26
#define L3GD20_STATUS_REG_ADDR        0x27
#define L3GD20_OUT_X_L_ADDR           0x28
#define L3GD20_OUT_X_H_ADDR           0x29
#define L3GD20_OUT_Y_L_ADDR           0x2a
#define L3GD20_OUT_Y_H_ADDR           0x2b
#define L3GD20_OUT_Z_L_ADDR           0x2c
#define L3GD20_OUT_Z_H_ADDR           0x2d
#define L3GD20_FIFO_CTRL_REG_ADDR     0x2e
#define L3GD20_FIFO_SRC_REG_ADDR      0x2f
#define L3GD20_INT1_CFG_ADDR          0x30
#define L3GD20_INT1_SRC_ADDR          0x31
#define L3GD20_INT1_TSH_XH_ADDR       0x32
#define L3GD20_INT1_TSH_XL_ADDR       0x33
#define L3GD20_INT1_TSH_YH_ADDR       0x34
#define L3GD20_INT1_TSH_YL_ADDR       0x35
#define L3GD20_INT1_TSH_ZH_ADDR       0x36
#define L3GD20_INT1_TSH_ZL_ADDR       0x37
#define L3GD20_INT1_DURATION_ADDR     0x38

#define L3GD20_CTRL_REG1_ENABLE_X     0x01
#define L3GD20_CTRL_REG1_ENABLE_Y     0x02
#define L3GD20_CTRL_REG1_ENABLE_Z     0x04
#define L3GD20_CTRL_REG1_POWER        0x08
#define L3GD20_CTRL_REG1_BW0          0x10
#define L3GD20_CTRL_REG1_BW1          0x20
#define L3GD20_CTRL_REG1_DR0          0x40
#define L3GD20_CTRL_REG1_DR1          0x80

#define L3GD20_CTRL_REG3_PP_OD        0x10

#define L3GD20_CTRL_REG4_SIM          0x01
#define L3GD20_CTRL_REG4_FS0          0x10
#define L3GD20_CTRL_REG4_FS1          0x20
#define L3GD20_CTRL_REG4_BLE          0x40
#define L3GD20_CTRL_REG4_BDU          0x80

#define L3GD20_CTRL_REG4_FS_250       0x00
#define L3GD20_CTRL_REG4_FS_500       L3GD20_CTRL_REG4_FS0
#define L3GD20_CTRL_REG4_FS_2000      L3GD20_CTRL_REG4_FS1

#define L3GD20_CTRL_REG5_OUT_SEL0     0x01
#define L3GD20_CTRL_REG5_OUT_SEL1     0x02
#define L3GD20_CTRL_REG5_IN_SEL0      0x04
#define L3GD20_CTRL_REG5_IN_SEL1      0x08
#define L3GD20_CTRL_REG5_HPEN         0x10
#define L3GD20_CTRL_REG5_FIFO_EN      0x40
#define L3GD20_CTRL_REG5_REBOOT       0x80

#define L3GD20_ID                     0xd4

struct l3gd20_xzy_data
{
	uint8_t reserved;
	uint8_t temp;
	uint8_t status;
	int16_t x;
	int16_t y;
	int16_t z;
} __attribute__ (( packed ));

static void l3gd20_task(void* arg);
static unsigned char l3gd20_rxbuffer[8];
static struct l3gd20_xzy_data l3gd20_data;
static const unsigned char l3gd20_xyz_data_txbuffer[sizeof(l3gd20_data)+1] = {L3GD20_OUT_TEMP_ADDR|L3GD20_READCMD|L3GD20_MULTIBYTECMD, 0x00};

int l3gd20_module_init()
{
	xTaskCreate(l3gd20_task, "l3gd20", L3GD20_STACK_SIZE, NULL, PRIORITY_TASK_GYRO, NULL);

	return 0;
}

module_init(l3gd20_module_init, INIT_GYRO);

static int l3gd20_init()
{
	unsigned char txbuffer[2] = {L3GD20_ID_ADDR|L3GD20_READCMD, 0x00};
	spi_transaction(SPI_DEVICE_GYRO, txbuffer, l3gd20_rxbuffer, 3);

	if( l3gd20_rxbuffer[1] == L3GD20_ID )
	{
		log_format(LOG_INFO, "l3gd20 found");
	}
	else
	{
		log_format(LOG_ERROR, "l3gd20 not found id %x expected %x", l3gd20_rxbuffer[1], L3GD20_ID);
		return -1;
	}

	txbuffer[0] = L3GD20_CTRL_REG1_ADDR;
	txbuffer[1] = L3GD20_CTRL_REG1_ENABLE_X | L3GD20_CTRL_REG1_ENABLE_Y | L3GD20_CTRL_REG1_ENABLE_Z | L3GD20_CTRL_REG1_POWER;
	spi_transaction(SPI_DEVICE_GYRO, txbuffer, l3gd20_rxbuffer, 2);

	txbuffer[0] = L3GD20_CTRL_REG4_ADDR;
	txbuffer[1] = L3GD20_CTRL_REG4_FS_500;
	spi_transaction(SPI_DEVICE_GYRO, txbuffer, l3gd20_rxbuffer, 2);

	txbuffer[0] = L3GD20_CTRL_REG2_ADDR;
	txbuffer[1] = 0;
	spi_transaction(SPI_DEVICE_GYRO, txbuffer, l3gd20_rxbuffer, 2);

	txbuffer[0] = L3GD20_CTRL_REG3_ADDR;
	txbuffer[1] = L3GD20_CTRL_REG3_PP_OD;
	spi_transaction(SPI_DEVICE_GYRO, txbuffer, l3gd20_rxbuffer, 2);

	txbuffer[0] = L3GD20_CTRL_REG5_ADDR;
	txbuffer[1] = L3GD20_CTRL_REG5_HPEN;
	spi_transaction(SPI_DEVICE_GYRO, txbuffer, l3gd20_rxbuffer, 2);

	return 0;
}

static void l3gd20_task(void* arg)
{
	(void) arg;
	int res = l3gd20_init();
	portTickType wakeTime = xTaskGetTickCount();
	int delta = 1;

	if( res )
	{
		// erreur d'init, on ne fait rien
		vTaskSuspend(NULL);
	}

	while(1)
	{
		spi_transaction(SPI_DEVICE_GYRO, l3gd20_xyz_data_txbuffer, &l3gd20_data, sizeof(l3gd20_data)+1);
		if( l3gd20_data.status != 0 )
		{
			//log_format(LOG_INFO, "xyz %5d %5d %5d temp %d°C status %x", l3gd20_data.x, l3gd20_data.y, l3gd20_data.z, l3gd20_data.temp, l3gd20_data.status);
			delta = 10;
		}
		else
		{
			delta = 1;
		}

		vTaskDelayUntil(&wakeTime, delta);
	}
}
