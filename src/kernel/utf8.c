//! @file serial_number.c
//! @brief Serial number of stm32 (96 bit)
//! @author Atlantronic

#include "kernel/utf8.h"

//! Conversion int => utf8 en hex
void uint_to_hex_utf8(uint32_t val, uint8_t *pbuf, uint8_t len)
{
	uint8_t idx = 0;

	for( idx = 0 ; idx < len ; idx ++)
	{
		if( ((val >> 28)) < 0xA )
		{
			pbuf[ 2* idx] = (val >> 28) + '0';
		}
		else
		{
			pbuf[2* idx] = (val >> 28) + 'A' - 10; 
		}

		val = val << 4;

		pbuf[ 2* idx + 1] = 0;
	}
}

