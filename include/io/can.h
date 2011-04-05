#ifndef CAN_H
#define CAN_H

//! @file can.h
//! @brief CAN
//! @author Jean-Baptiste Trédez

#include "cpu/cpu.h"

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
	unsigned int id; //!< 11 bits ou 29 bits si étendue
	unsigned char data[8]; //!< données (de 0 à  8 octets)
	unsigned char size; //!< taille
	unsigned char format; //!< format (standard ou étendu)
	unsigned char type; //!< type
};

#endif