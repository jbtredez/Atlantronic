#ifndef CAN_H
#define CAN_H

//! @file can.h
//! @brief CAN
//! @author Jean-Baptiste Trédez

#include "kernel/cpu/cpu.h"

enum can_format
{
	CAN_STANDARD_FORMAT,
	CAN_EXTENDED_FORMAT
};

enum can_type
{
	CAN_DATA_FRAME,
	CAN_REMOTE_FRAME
};

struct can_msg
{
	uint32_t id; //!< 11 bits ou 29 bits si étendue
	// accès aux données par tableau ou par 2 mots de 32 bits (pour le driver)
	union
	{
		uint8_t data[8]; //!< données (de 0 à  8 octets)
		struct {
			uint32_t low;
			uint32_t high;
		} _data;
	};
	unsigned char size; //!< taille
	unsigned char format; //!< format (standard ou étendu)
	unsigned char type; //!< type
};

#endif
