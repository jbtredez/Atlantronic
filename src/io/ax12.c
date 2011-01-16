//! @file ax12.c
//! @brief Gestion AX12
//! @author Jean-Baptiste Trédez

#include "io/ax12.h"
#include "module.h"

#define AX12_INSTRUCTION_PING             0x01
#define AX12_INSTRUCTION_READ_DATA        0x02
#define AX12_INSTRUCTION_WRITE_DATA       0x03
#define AX12_INSTRUCTION_REG_WRITE        0x04
#define AX12_INSTRUCTION_ACTION           0x05
#define AX12_INSTRUCTION_RESET            0x06
#define AX12_INSTRUCTION_SYNC_WRITE       0x83

static uint8_t ax12_checksum(uint8_t* buffer, uint8_t size);

static int ax12_module_init()
{
	ax12_ping(22);

	return 0;
}

module_init(ax12_module_init, INIT_AX12);

void ax12_ping(uint8_t id)
{
	uint8_t buffer[5];
	buffer[0] = 0xFF;
	buffer[1] = 0xFF;
	buffer[2] = id;
	buffer[3] = 0x02;
	buffer[4] = ax12_checksum(buffer, sizeof(buffer));

	usart_write(buffer, sizeof(buffer));
}

static uint8_t ax12_checksum(uint8_t* buffer, uint8_t size)
{
	uint8_t i = 2;
	uint8_t checksum = 0;

	for(; i< size - 1 ; i++)
	{
		checksum += buffer[i];
	}
	checksum = ~checksum;

	return checksum;
}
