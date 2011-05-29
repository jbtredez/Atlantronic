//! @file hokuyo_tools.c
//! @brief Hokuyo tools
//! @author Atlantronic

#include "kernel/hokuyo_tools.h"
#include "kernel/robot_parameters.h"

uint16_t hokuyo_tools_decode16(const unsigned char* data)
{
	uint16_t val = *data++ - 0x30;
	val <<= 6;
	val &= ~0x3f;
	val |= *data - 0x30;

	return val;
}

int hokuyo_tools_decode_buffer(const unsigned char* buffer, unsigned int buffer_size, uint16_t* distance, unsigned int distance_size)
{
	int j = 0;
	int i = 0;
	uint8_t sum = 0;
	int res = 0;
	unsigned int num_pack;
	unsigned int num_last_data;

	if( buffer_size < 23)
	{
		// TODO ERR code
		res = -1;
		goto end;
	}

	// on passe l'entête
	buffer += 23;
	buffer_size -= 23;

	num_pack = (buffer_size - 1) / 66;
	num_last_data = (buffer_size - 3 - num_pack * 66) >> 1;

	if( distance_size < 32 * num_pack + num_last_data)
	{
		// TODO ERR code
		res = -2;
		goto end;
	}

	// traitement des pack de 64 data + sum + LF
	for( i = num_pack; i--; )
	{
		sum = 0;
		for( j = 32 ; j-- ; )
		{
			*distance = hokuyo_tools_decode16(buffer);
			distance++;
			sum += *buffer;
			buffer++;
			sum += *buffer;
			buffer++;
		}

		sum &= 0x3F;
		sum += 0x30;

		if( sum != *buffer)
		{
			// erreur checksum
			// TODO led
			// par sécurité, on met le code d'erreur 10
			distance -= 32;
			for( j = 32 ; j-- ; )
			{
				*distance = 10;
				distance++;
			}
			res = -1;
		}
		buffer+=2;
	}

	// traitement du reste
	for(  ; num_last_data-- ; buffer += 2)
	{
		*distance = hokuyo_tools_decode16(buffer);
		distance++;
	}

end:
	return res;
}

void hokuyo_compute_xy(uint16_t* distance, unsigned int size, float* x, float* y)
{
	float alpha = -(135 / 180.0f - 44 / 512.0f) * PI;
	const float pas = PI / 512.0f;

	for( ; size--; )
	{
		if(*distance > 19)
		{
			*x = *distance * cos(alpha);
			*y = *distance * sin(alpha);
		}
		else
		{
			*x = 0;
			*y = 0;
		}

		distance++;
		x++;
		y++;
		alpha += pas;
	}
}
