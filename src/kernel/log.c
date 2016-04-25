//! @file log.c
//! @brief Log Task
//! @author Atlantronic

#define NO_WEAK_LOG
#include "kernel/log.h"
#undef NO_WEAK_LOG
#include <stdarg.h>
#include "kernel/driver/xbee.h"
#include "kernel/driver/ESP8266.h"

//! attention, coute tres cher en stack
void log_format_and_add(unsigned char level, const char* func, uint16_t line, const char* msg, ...)
{
	char buffer[LOG_SIZE];

	va_list ap;
	va_start(ap, msg);
	vsnprintf(buffer, LOG_SIZE, msg, ap);
	va_end(ap);

	usb_add_log(level, func, line, buffer);
	xbee_add_log(level, func, line, buffer);
	//esp8266_add_log(level, func, line, buffer);
}

void log_add(unsigned char level, const char* func, uint16_t line, const char* msg)
{
	usb_add_log(level, func, line, msg);
	xbee_add_log(level, func, line, msg);
	//esp8266_add_log(level, func, line, msg);
}
