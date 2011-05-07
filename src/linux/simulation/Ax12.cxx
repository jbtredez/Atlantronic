//! @file Ax12.cxx
//! @brief Ax12
//! @author Atlantronic

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
	send_buffer_size = 0;
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

uint8_t Ax12::usart_write()
{
	uint8_t rep = send_buffer[send_buffer_count];
	send_buffer_count++;
	return rep;
}

bool Ax12::usart_write_request()
{
	return send_buffer_count < send_buffer_size;
}

uint8_t Ax12::checksum(uint8_t* buf, uint8_t size)
{
	uint8_t i = 2;
	uint8_t checksum = 0;

	for(; i< size - 1 ; i++)
	{
		checksum += buf[i];
	}
	checksum = ~checksum;

	return checksum;
}

void Ax12::sendStatus()
{
	send_buffer_size = 6;
	send_buffer_count = 0;

	send_buffer[0] = 0xff;
	send_buffer[1] = 0xff;
	send_buffer[2] = id;
	send_buffer[3] = 0x02;
	send_buffer[4] = 0x00;
	send_buffer[5] = checksum(send_buffer, send_buffer_size);
}

void Ax12::process_write_data()
{
	meslog(_info_, "ax12 %i, write", id);

	uint8_t write_offset = msg[5];
	uint8_t write_size = msg_size - 7;
	uint8_t* buf = msg + 6;

	if(write_offset + write_size <= sizeof(control_table))
	{
		for( ; write_size--; )
		{
			control_table[write_offset] = *buf;
			buf++;
			write_offset++;
		}
	}
	else
	{
		meslog(_erreur_, "ecriture en dehors des limites : offset %#.2x, taille : %#.2x", write_offset, write_size);
	}

	if(msg[2] != 0xFE)
	{
		sendStatus();
	}
}

void Ax12::process_read_data()
{
	meslog(_info_, "ax12 %i, read", id);

	unsigned int read_offset = msg[5];
	unsigned int argc = msg[6];

	if(read_offset + argc > sizeof(control_table))
	{
		meslog(_erreur_, "lecture en dehors des limites : offset %#.2x, taille : %#.2x", read_offset, argc);
		sendStatus();
		return;
	}

	send_buffer_size = 6 + argc;
	send_buffer_count = 0;

	send_buffer[0] = 0xff;
	send_buffer[1] = 0xff;
	send_buffer[2] = id;
	send_buffer[3] = 0x02 + argc;
	send_buffer[4] = 0x00;
	for( int i = 5; i < send_buffer_size - 1 ; i++)
	{
		send_buffer[i] = control_table[read_offset];
		read_offset++;
	}
	send_buffer[send_buffer_size - 1] = checksum(send_buffer, send_buffer_size);
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

	// on a reçu tout le message, est-ce qu'il est pour nous ?
	if(msg[2] != id && msg[2] != 0xFE)
	{
		msg_size = 0;
		msg_expected_size = 4;
		return;
	}

	uint8_t chksum = checksum(msg, msg_size);
	if( chksum != msg[msg_size-1])
	{
		msg_size = 0;
		msg_expected_size = 4;
		meslog(_erreur_, "checksum : reçu = %#.2x, cal = %#.2x", msg[msg_size-1], chksum);
		return;
	}

	switch(msg[4])
	{
		case AX12_INSTRUCTION_PING:
			meslog(_info_, "ax12 %i, ping", id);
			if(msg[2] != 0xFE)
			{
				sendStatus();
			}
			break;
		case AX12_INSTRUCTION_READ_DATA:
			process_read_data();
			break;
		case AX12_INSTRUCTION_WRITE_DATA:
			process_write_data();
			break;
		case AX12_INSTRUCTION_REG_WRITE:
			meslog(_info_, "ax12 %i, reg write", id);
			meslog(_erreur_, "ax12 %i, reg write : pas implémenté", id);
			// TODO
			if(msg[2] != 0xFE)
			{
				sendStatus();
			}
			break;
		case AX12_INSTRUCTION_ACTION:
			meslog(_info_, "ax12 %i, action", id);
			meslog(_erreur_, "ax12 %i, action : pas implémenté", id);
			// TODO	
			if(msg[2] != 0xFE)
			{
				sendStatus();
			}
			break;
		case AX12_INSTRUCTION_RESET:
			meslog(_info_, "ax12 %i, reset", id);
			if(msg[2] != 0xFE)
			{
				sendStatus();
			}
			break;
		case AX12_INSTRUCTION_SYNC_WRITE:
			meslog(_info_, "ax12 %i, sync write", id);
			if(msg[2] != 0xFE)
			{
				meslog(_erreur_, "AX12_INSTRUCTION_SYNC_WRITE mais id != 0xFE");
			}
			meslog(_erreur_, "ax12 %i, sync write : pas implémenté", id);
			// TODO
			break;
		default:
			meslog(_erreur_, "instrucion inconnue");
			break;
	}

	msg_size = 0;
	msg_expected_size = 4;
}
