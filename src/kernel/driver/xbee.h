#ifndef XBEE_H
#define XBEE_H

//! @file xbee.h
//! @brief Xbee module
//! @author Atlantronic

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef WEAK_XBEE
#define WEAK_XBEE __attribute__((weak, alias("nop_function") ))
#endif

#define XBEE_CMD_AT                        0x08      //!< commande AT
#define XBEE_CMD_AT_QUEUE_PARAM_VALUE      0x09      //!< commande AT avec effet retarde
#define XBEE_CMD_TX                        0x10      //!< envoi
#define XBEE_CMD_REMOTE_AT                 0x17
#define XBEE_CMD_RX_MASK                   0x80
#define XBEE_CMD_MODEM_STATUS              0x8a

#define XBEE_AT_BAUDRATE                   (('B' << 8) + 'D')
#define XBEE_AT_WRITE_EEPROM               (('W' << 8) + 'R')
#define XBEE_AT_SW_RESET                   (('F' << 8) + 'R')
#define XBEE_AT_NETWORK_ID                 (('I' << 8) + 'D')

#define XBEE_OP_BAUDRATE                   115200
#define XBEE_TIMEOUT                           50
#define XBEE_NETWORK_ID                    0x1818
#define XBEE_ADDR_DISCOVERY_H            0x13a200
#define XBEE_ADDR_DISCOVERY_L          0x40991d2f
#define XBEE_ADDR_PC_H                   0x13a200
#define XBEE_ADDR_PC_L                 0x409e0da5

typedef enum
{
	XBEE_STATUS_DISCONNECTED = 0,
	XBEE_STATUS_CONNECTED,
} XbeeStatus;

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

void xbee_add(uint16_t type, void* msg, uint16_t size) WEAK_XBEE;

void xbee_add_log(unsigned char level, const char* func, uint16_t line, const char* msg) WEAK_XBEE;

#ifdef __cplusplus
}
#endif

#endif
