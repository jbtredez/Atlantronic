//! @file log.c
//! @brief Log Task
//! @author Atlantronic

#define NO_WEAK_LOG
#include "kernel/log.h"
#undef NO_WEAK_LOG
#include <stdarg.h>
#include "kernel/driver/usb.h"

//! attention, coute tres cher en stack
void log_format_and_add(unsigned char level, const char* func, uint16_t line, const char* msg, ...)
{
	char buffer[LOG_SIZE];

	uint64_t current_time = systick_get_time();
	memcpy(buffer, &current_time, 8);
	buffer[8] = level;
	memcpy(buffer+9, &line, 2);

	int len = strlen(func);

	if(len > 20)
	{
		len = 20;
	}

	memcpy(buffer + 11, func, len);
	len += 11;
	buffer[len] = ':';
	len++;

	va_list ap;
	va_start(ap, msg);
	char len2 = vsnprintf(buffer + len, LOG_SIZE-len - 1, msg, ap);
	va_end(ap);

	len += len2;
	buffer[len] = '\n';
	len++;

	usb_add(USB_LOG, (unsigned char*) buffer, len);
}

void log_add(unsigned char level, const char* func, uint16_t line, const char* msg)
{
	char buffer[LOG_SIZE];

	uint64_t current_time = systick_get_time();
	memcpy(buffer, &current_time, 8);
	buffer[8] = level;
	memcpy(buffer+9, &line, 2);

	int len = strlen(func);

	if(len > 20)
	{
		len = 20;
	}

	memcpy(buffer + 11, func, len);
	len += 11;
	buffer[len] = ':';
	len++;

	int len2 = strlen(msg);
	if(len + len2 >= LOG_SIZE)
	{
		len2 = LOG_SIZE - len - 1;
	}

	memcpy(buffer + len, msg, len2);
	len += len2;
	buffer[len] = '\n';
	len++;

	usb_add(USB_LOG, (unsigned char*) buffer, len);
}