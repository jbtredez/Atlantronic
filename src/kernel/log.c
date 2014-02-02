//! @file log.c
//! @brief Log Task
//! @author Atlantronic

#define NO_WEAK_LOG
#include "kernel/log.h"
#undef NO_WEAK_LOG
#include <stdarg.h>

//! attention, coute tres cher en stack
void log_format_and_add(unsigned char level, const char* func, uint16_t line, const char* msg, ...)
{
	char buffer[LOG_SIZE];

	va_list ap;
	va_start(ap, msg);
	vsnprintf(buffer, LOG_SIZE, msg, ap);
	va_end(ap);

	usb_add_log(level, func, line, buffer);
}

void log_add(unsigned char level, const char* func, uint16_t line, const char* msg)
{
	usb_add_log(level, func, line, msg);
}
