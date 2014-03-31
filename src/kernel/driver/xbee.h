#ifndef XBEE_H
#define XBEE_H

//! @file xbee.h
//! @brief Xbee module
//! @author Atlantronic


#include <stdint.h>

enum
{
	XBEE_CMD_SET_MANAGER_BAUDRATE = 1,
	XBEE_CMD_SET_OP_BAUDRATE,
	XBEE_CMD_SET_OP_CONFIGURATION,
};

struct xbee_cmd_param
{
	uint8_t cmd_id;         //!< id de la commande
	float param;            //!< parametre
} __attribute((packed));

#endif
