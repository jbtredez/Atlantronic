//! @file Ax12.cxx
//! @brief Ax12
//! @author Jean-Baptiste Trédez

#include "Ax12.h"
#include "log.h"

#define AX12_INSTRUCTION_PING             0x01
#define AX12_INSTRUCTION_READ_DATA        0x02
#define AX12_INSTRUCTION_WRITE_DATA       0x03
#define AX12_INSTRUCTION_REG_WRITE        0x04
#define AX12_INSTRUCTION_ACTION           0x05
#define AX12_INSTRUCTION_RESET            0x06
#define AX12_INSTRUCTION_SYNC_WRITE       0x83

uint8_t Ax12::control_table_init[50] = {
	0x0C, // Model number (L)
	0x00, // Model number (H)
	0x00, // Version of firmware
	0x01, // ID
	0x01, // Baud rate
	0xFA, // Return delay time
	0x00, // CW angle limit (L)
	0x00, // CW angle limit (H)
	0xFF, // CCW angle limit (L)
	0x03, // CCW angle limit (H)
	0x00, // (Reserved)
	0x55, // Highest limit temperature
	0x3C, // Lowest limit voltage
	0xBE, // Highest limit voltage
	0xFF, // Max Torque (L)
	0x03, // Max Torque (H)
	0x02, // Status return level
	0x04, // Alarm LED
	0x04, // Alarm shutdown
	0x00, // (Reserved)
	0x00, // Down calibration (L)
	0x00, // Down calibration (H)
	0x00, // Up calibration (L)
	0x00, // Up calibration (H)
	0x00, // Torque enable
	0x00, // LED
	0x00, // CW compliance margin
	0x00, // CCW compliance margin
	0x20, // CW compliance slope
	0x20, // CCW compliance Slope
	0x00, // Goal position (L)
	0x00, // Goal position (H)
	0x00, // Moving speed (L)
	0x00, // Moving speed (H)
	0xFF, // Torque limit (L)
	0x03, // Torque limit (H)
	0x00, // Present position (L)
	0x00, // Present position (H)
	0x00, // Present speed (L)
	0x00, // Present speed (H)
	0x00, // Present Load (L)
	0x00, // Present Load (H)
	0x64, // Present voltage
	0x14, // Present temperature
	0x00, // Registered Instruction
	0x00, // (reserved)
	0x00, // Moving
	0x00, // Lock
	0x20, // Punch (L)
	0x00  // Punch (H)
};

Ax12::Ax12(uint8_t Id) :
	id(Id)
{
	msg_size = 0;
	msg_expected_size = 4;
	memcpy(control_table, control_table_init, sizeof(control_table_init));
}

Ax12::~Ax12()
{

}

void Ax12::usart_read(uint8_t octet)
{
	msg[msg_size] = octet;
	msg_size++;
	if(msg_size == msg_expected_size)
	{
		process_msg();
	}
}

uint8_t Ax12::checksum()
{
	uint8_t i = 2;
	uint8_t checksum = 0;

	for(; i< msg_size - 1 ; i++)
	{
		checksum += msg[i];
	}
	checksum = ~checksum;

	return checksum;
}

void Ax12::process_msg()
{
	if(msg[0] != 0xFF || msg[1] != 0xFF)
	{
		meslog(_erreur_, "erreur de protocole");
		msg_size = 0;
		msg_expected_size = 4;
		return;
	}

	msg_expected_size = msg[3] + 4;
	if(msg_expected_size != msg_size)
	{
		return;
	}

	if(msg[2] != id && msg[2] != 0xFE)
	{
		msg_size = 0;
		msg_expected_size = 4;
		return;
	}

	if(checksum() != msg[msg_size-1])
	{
		msg_size = 0;
		msg_expected_size = 4;
		meslog(_erreur_, "checksum");
		return;
	}

	switch(msg[4])
	{
		case AX12_INSTRUCTION_PING:
			meslog(_info_, "ax12 %i, ping", id);
			// TODO			
			break;
		case AX12_INSTRUCTION_READ_DATA:
			meslog(_info_, "ax12 %i, read %#.2x - %#.2x", id, msg[5], msg[6]);
			// TODO
			break;
		case AX12_INSTRUCTION_ACTION:
			meslog(_info_, "ax12 %i, action", id);
			// TODO	
			break;
		case AX12_INSTRUCTION_RESET:
			meslog(_info_, "ax12 %i, reset", id);
			// TODO	
			break;
		default:
			meslog(_erreur_, "instrucion inconnue");
			break;
	}

	msg_size = 0;
	msg_expected_size = 4;
}
