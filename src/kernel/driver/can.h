#ifndef CAN_H
#define CAN_H

//! @file can.h
//! @brief CAN
//! @author Atlantronic

#ifndef LINUX
#include "kernel/cpu/cpu.h"
#include "kernel/portmacro.h"
#include "kernel/FreeRTOS.h"
#include "kernel/task.h"
#include "kernel/queue.h"
#endif
#include "kernel/systick.h"

#ifdef __cplusplus
extern "C" {
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
	struct systime time;
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

#ifndef LINUX
int can_open(enum can_baudrate baudrate, xQueueHandle _can_read_queue);

uint32_t can_write(struct can_msg *msg, portTickType timeout);

#endif

#ifdef __cplusplus
}
#endif

#endif
