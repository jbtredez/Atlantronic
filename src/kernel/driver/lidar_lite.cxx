#include "lidar_lite.h"
#include "kernel/driver/i2c.h"
#include "kernel/module.h"
#include "kernel/log.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#define LIDAR_LITE_ADDR                   0x62 // adresse i2c
#define LIDAR_LITE_TIMEOUT                   1

#define LIDAR_LITE_REG_COMMAND            0x00
#define LIDAR_LITE_REG_MODE_STATUS        0x01
#define LIDAR_LITE_REG_HIGH_LOWB          0x8f

#define LIDAR_LITE_REG_COMMAND_RESET                                  0x00
#define LIDAR_LITE_REG_COMMAND_CORRELATION_ONLY                       0x01
#define LIDAR_LITE_REG_COMMAND_REPROCESS                              0x02
#define LIDAR_LITE_REG_COMMAND_MES_AND_CORRELATION                    0x03
#define LIDAR_LITE_REG_COMMAND_MES_AND_CORRELATION_DC_CORRECTION      0x04

LidarLite::LidarLite()
{
	m_distance = 0;
}

static int toto = 0;
void LidarLite::update()
{
	toto++;
	if( toto == 200)
	{
		uint8_t tx_buffer[2];

		tx_buffer[0] = LIDAR_LITE_REG_COMMAND;
		tx_buffer[1] = LIDAR_LITE_REG_COMMAND_MES_AND_CORRELATION_DC_CORRECTION;
		i2c_write_data(LIDAR_LITE_ADDR, tx_buffer, 2, LIDAR_LITE_TIMEOUT);
	}
	if( toto > 200 )
	{
		uint8_t tx_buffer = LIDAR_LITE_REG_HIGH_LOWB;
		uint8_t rx_buffer[2];
		int err = i2c_transaction(LIDAR_LITE_ADDR, &tx_buffer, 1, &rx_buffer, 2, LIDAR_LITE_TIMEOUT);
		if( ! err )
		{
			m_distance = 10 * ((((uint16_t)rx_buffer[0]) << 8) + (uint16_t)rx_buffer[1]);
			log_format(LOG_INFO, "distance %d", (int)m_distance);
			toto = 100;
		}
		if( err == I2C_ERROR_TIMEOUT )
		{
			toto = 100;
		}
	}
}
