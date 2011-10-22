//! @file log.c
//! @brief Log Task
//! @author Atlantronic

#define NO_WEAK_LOG
#include "kernel/log.h"
#undef NO_WEAK_LOG
#include <stdarg.h>
#include "kernel/driver/usb.h"

//! attention, coute tres cher en stack
void log_format_and_add(const char* msg, ...)
{
	char buffer[LOG_SIZE];

	va_list ap;
	va_start(ap, msg);
	char size = vsnprintf(buffer, LOG_SIZE, msg, ap);
	va_end(ap);

	usb_add(USB_LOG, (unsigned char*) buffer, size);
}
