#ifndef ESP8266_H
#define ESP8266_H

//! @file esp8266.h
//! @brief ESP8266 module
//! @author Atlantronic

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef WEAK_ESP8266
#define WEAK_ESP8266 __attribute__((weak, alias("nop_function") ))
#endif

#define ESP8266_CMD_WRITE           0x02
#define ESP8266_CMD_READ			0x03      //!< commande AT avec effet retarde

typedef enum
{
	ESP8266_STATUS_DISCONNECTED = 0,
	ESP8266_STATUS_CONNECTED,
} ESP8266Status;



struct esp8266_cmd_param
{
	uint8_t cmd_id;         //!< id de la commande
	float param;            //!< parametre
} __attribute((packed));

void esp8266_add(uint16_t type, void* msg, uint16_t size) WEAK_ESP8266;

void esp82666_add_log(unsigned char level, const char* func, uint16_t line, const char* msg) WEAK_ESP8266;

#ifdef __cplusplus
}
#endif

#endif
