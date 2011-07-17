#ifndef LOG_H
#define LOG_H

//! @file log.h
//! @brief Log task
//! @author Atlantronic

#include "kernel/systick.h"
#include "kernel/portmacro.h"
#include <stdio.h>
#include <errno.h>
#include <string.h>

// taille max d'un log (espace qui doit être dispo sur la stack)
#define LOG_SIZE             100

void log_format_and_add(const char* msg, ...) __attribute__(( format(printf, 1, 2) ));

#define log_error(msg, arg ...) \
	do \
	{ \
		log_format_and_add("%12lu\tError\t%10s:%i\t"msg"\n", (unsigned long int)tick_to_us( systick_get_time() ), __FUNCTION__, __LINE__, ##arg);\
	}while(0)

#endif
