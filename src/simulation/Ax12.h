#ifndef AX12_H
#define AX12_H

//! @file Ax12.h
//! @brief Ax12
//! @author Jean-Baptiste Trédez

#include <stdint.h>
#include "UsartDevice.h"

class Ax12 : public UsartDevice
{
public:
	Ax12(uint8_t id);
	~Ax12();

protected:
	void usart_read(uint8_t octet);
	void process_msg();
	uint8_t checksum();

	uint8_t id;
	uint8_t msg[256];
	uint8_t msg_size;
	uint8_t msg_expected_size;
	uint8_t control_table[50];
	static uint8_t control_table_init[50];
};

#endif
