#ifndef CAN_H
#define CAN_H

//! @file can.h
//! @brief CAN
//! @author Atlantronic

#ifndef LINUX
#include "kernel/cpu/cpu.h"
#include "kernel/portmacro.h"
#else
typedef uint64_t portTickType;
#endif

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

enum can_baudrate
{
	CAN_1000,
	CAN_800,
	CAN_500,
	CAN_250,
	CAN_125,
	CAN_RESERVED,
	CAN_50,
	CAN_20,
	CAN_10
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
} __attribute__((packed));

typedef void (*can_callback)(struct can_msg *msg);

uint32_t can_write(struct can_msg *msg, portTickType timeout);

//! enregistre la fonction pour qu'elle soit appellée si un message avec l'identifiant "id" au format "format" est reçu
//! le message est démasqué sur le filtre matériel du can
uint32_t can_register(uint32_t id, enum can_format format, can_callback function);

#endif
