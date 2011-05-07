#ifndef AX12_H
#define AX12_H

//! @file Ax12.h
//! @brief Ax12
//! @author Atlantronic

#include <stdint.h>
#include "UsartDevice.h"

class Ax12 : public UsartDevice
{
public:
	Ax12(uint8_t id);
	~Ax12();

protected:
	void usart_read(uint8_t octet);
	uint8_t usart_write();
	void process_msg();
	void process_write_data();
	void process_read_data();
	uint8_t checksum(uint8_t* buf, uint8_t size);
	bool usart_write_request();
	void sendStatus();

	uint8_t id;
	uint8_t msg[256];
	uint8_t msg_size;
	uint8_t msg_expected_size;
	uint8_t control_table[50];
	static uint8_t control_table_init[50];
	uint8_t send_buffer[256];
	uint8_t send_buffer_size;
	uint8_t send_buffer_count;
};

#endif
